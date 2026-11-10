from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, structs, make_tester_present_msg
from opendbc.car.lateral import apply_steer_angle_limits_vm, get_max_angle_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import (create_lka_steering, create_resume_acc, create_disable_radar, create_enable_radar,
                                    create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6)
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.psa.values import CarControllerParams, PSAFlags
from opendbc.car.vehicle_model import VehicleModel
from numpy import interp
import math

try:
  from openpilot.cereal import messaging
  sm =messaging.SubMaster(['modelV2'], poll='modelV2')
except ImportError:
  # cereal is only available in openpilot, not in standalone opendbc
  sm = None

LongCtrlState = structs.CarControl.Actuators.LongControlState

# pandad leaves the panda in ELM327 mode until card publishes CarParams, measured
# at 20-174 ms after the first control frame across 20 logged drives. The radar
# knockout goes to a diagnostic address, which ELM327 lets through, but the
# emulated 0x2B6/0x2F6 that take the radar's place are rejected until the switch
# to the PSA safety mode lands. Knocking the radar out inside that window leaves
# the ADAS bus silent, and the ESP (UC_FREIN) marks its ACC fields invalid after
# ~150 ms of silence, which openpilot then reports as accFaulted. Hold off well
# past the worst measured delay before touching the radar.
RADAR_DISABLE_FRAME = 100  # 1.0 s

# The radar resumed 220-230 ms after the programming session ended in the logs, but that
# was via the S3 timeout, so allow generous margin. The release ends as soon as the real
# 0x2B6 reappears, so this only bounds the case where it never does.
RADAR_ENABLE_TIMEOUT_FRAMES = 200  # 2.0 s

# The ESP holds the car at a standstill for the radar (autohold-style) and reports it as
# ARRET_VHL_ADAS in 0x32D, about a second after the car comes to rest. What releases that
# hold is the launch pattern in 0x2B6, not the 0x2F6 pulse: across the stock drive-aways
# the ESP cleared ARRET_VHL_ADAS 70-100 ms after the first launch frame, then ramped the
# brakes out over ~0.5 s, and one stock launch carried no DRIVE_AWAY_REQUEST at all. The
# pulse is kept because the stock radar usually sends one, but only for its measured
# length of ~40 ms — see create_HS2_DYN1_MDD_ETAT_2B6.
DRIVE_AWAY_FRAMES = 4   # 40 ms at 100 Hz
LAUNCH_COMPLETE_SPEED = 0.5  # m/s, decel request released above this

# CMM (the engine ECU) takes a wheel-torque request, not an acceleration, so this map is
# the conversion the stock radar would have done, for the pitch compensated accel. It is
# speed independent: drag and the launch are left to the longitudinal integrator, see kiV
# in interface.py. helper-scripts/accel_map.py imports these directly so its suggestions
# can never be against a stale copy.
ACCEL_LOOKUP = [-1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 3.0]         # m/s²
# from 208 and Corsa F drives
TORQUE_LOOKUP = [-400, -140, 82, 375, 615, 865, 1100, 1600]       # N.m
# Calibrated on an e-208 against measured accel from the LongitudinalManeuverMode suite; holding
# 0 m/s² took ~170 N.m and delivering -0.5 m/s² took ~-130 N.m, so unlike the map above this one
# has typical drag built in. The 3.0 point extends the measured 434 N.m per m/s² slope past 2.0.
TORQUE_LOOKUP_EV = [-400, -130, 170, 350, 645, 862, 1100, 1534]   # N.m


def get_safety_CP():
  # We use the PSA_PEUGEOT_208 platform for lateral limiting to match safety
  from opendbc.car.psa.interface import CarInterface
  return CarInterface.get_non_essential_params("PSA_PEUGEOT_208")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_angle_last = 0
    self.lat_active_last = False
    self.engage_frame = 0
    self.radar_disable_sent = False
    self.radar_disabled = False
    self.radar_release_requested = False
    self.radar_released = False
    self.radar_release_frame = 0
    self.status = 2
    self.bars = 4
    self.hold = False
    self.launching = False
    self.drive_away_frames = 0
    self.braking = False
    self.ev = bool(CP.flags & PSAFlags.EV)
    # the body pitches with the car's own accel, unfiltered that feeds back into the grade compensation
    self.pitch = FirstOrderFilter(0., 1.0, DT_CTRL)
    self.torque_last = None
    self.long_frames = 0

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

  def release_radar(self) -> bool:
    """Put the radar ECU back on the ADAS bus, called once alpha long is switched off.

    Dropping straight off the bus leaves the radar in its programming session until the
    S3 timer expires ~5 s later, and the ESP (UC_FREIN) marks its ACC fields invalid
    after ~150 ms of that, which openpilot reports as accFaulted. This has to run while
    the car is still onroad: pandad puts the panda in NO_OUTPUT the moment deviceState
    goes offroad, and everything we send after that is rejected.

    Non-blocking, the work happens in update(). Returns True once the radar is back.
    """
    if not self.CP.openpilotLongitudinalControl:
      return True  # nothing was ever knocked out
    self.radar_release_requested = True
    return self.radar_released

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    # longitudinal
    # starting = actuators.longControlState == LongCtrlState.starting and CS.out.vEgo <= self.CP.vEgoStarting
    # stopping = actuators.longControlState == LongCtrlState.stopping

    # lateral control
    apply_angle = actuators.steeringAngleDeg

    # don't take over a curve steered beyond the lateral accel limit: the limiter would clip
    # the command below the wheel, and safety blocks the unwind from there
    max_angle = get_max_angle_vm(max(CS.out.vEgoRaw, 1), self.VM, CarControllerParams)
    lat_active = CC.latActive and (self.lat_active_last or abs(CS.out.steeringAngleDeg) < max_angle)

    # smooth engagement: blend from current wheel angle to commanded angle over ~1.0s
    # on rising edge of lat_active to avoid a lunge when the initial command is far from the wheel
    ENGAGE_FRAMES = 100
    if lat_active and not self.lat_active_last:
      self.engage_frame = 0
    if lat_active and self.engage_frame < ENGAGE_FRAMES:
      self.engage_frame += 1
      blend = self.engage_frame / ENGAGE_FRAMES
      apply_angle = blend * apply_angle + (1 - blend) * CS.out.steeringAngleDeg
    self.lat_active_last = lat_active

    # low-pass filter at low speeds to suppress high-frequency jitter at low speeds.
    # the reason for this happening is unknown, but it may be related to the EPS torque sensor noise or quantization.
    # the filter time constant is reduced as speed increases to avoid excessive delay at higher speeds.
    if lat_active and CS.out.vEgoRaw < 5.0:
      tau = interp(CS.out.vEgoRaw, [0.5, 5.0], [0.3, 0.1])
      alpha = 1 - math.exp(-DT_CTRL / tau)
      apply_angle = alpha * apply_angle + (1 - alpha) * self.apply_angle_last

    apply_angle = apply_steer_angle_limits_vm(apply_angle, self.apply_angle_last, CS.out.vEgoRaw,
                                              CS.out.steeringAngleDeg, lat_active, CarControllerParams, self.VM)

    # the EPS disengages on driver override, re-engage with READY -> AUTHORIZED -> ACTIVE
    # the 508 EPS needs each step held, and ACTIVE held while it reports authorized
    # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
    if not lat_active:
      self.status = 2
    elif CS.eps_state in (1, 3):  # EPS_STATE_LKA authorized, active
      self.status = 4
    elif self.frame % CarControllerParams.STATUS_STEP == 0:
      self.status = 2 if self.status == 4 else self.status + 1

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])
    accel_slope = math.sin(self.pitch.x) * 9.81

    # the plan goes out unfiltered: a low-pass here sat inside the planner's loop, 1.6-2.7 s of lag
    # between the request and the torque swung the gap to a lead by ~15 m
    accel = min(max(actuators.accel, CarControllerParams.ACCEL_MIN), CarControllerParams.ACCEL_MAX)
    accel_cmd = accel + accel_slope

    # hysteresis so a request hovering at the threshold doesn't flip between torque and friction brakes
    P = CarControllerParams
    brake_on, brake_off = (P.EV_BRAKE_ON_ACCEL, P.EV_BRAKE_OFF_ACCEL) if self.ev else (P.BRAKE_ON_ACCEL, P.BRAKE_OFF_ACCEL)
    brake_accel = brake_off if self.braking else brake_on

    torque_nm = interp(accel_cmd, ACCEL_LOOKUP, TORQUE_LOOKUP_EV if self.ev else TORQUE_LOOKUP)
    torque = float(max(P.TORQUE_MIN, min(torque_nm, P.TORQUE_MAX)))

    braking = accel_cmd < brake_accel and not CS.out.gasPressed
    self.braking = braking
    if self.CP.openpilotLongitudinalControl:
      if CC.hudControl.leadVisible and sm is not None:
        sm.update(0)
        leads_v3 = sm['modelV2'].leadsV3
        if leads_v3 and leads_v3[0].x:
          r = leads_v3[0].x[0] / (3 + CS.out.vEgo)
          if self.bars > 3:  # initialize from "no lead"
            self.bars = min(3, int(r))
          elif r > self.bars + 1.2:
            self.bars = min(3, self.bars + 1)
          elif r < self.bars - 0.2:
            self.bars = max(0, self.bars - 1)
      else:
        self.bars = 4

      if self.radar_release_requested:
        # hand the bus back, see release_radar()
        if not self.radar_released:
          if not self.radar_disable_sent:
            self.radar_released = True  # never knocked it out, nothing to hand back
          else:
            if self.radar_release_frame == 0:
              can_sends.append(create_enable_radar())
            self.radar_release_frame += 1
            self.radar_released = CS.radar_alive or self.radar_release_frame >= RADAR_ENABLE_TIMEOUT_FRAMES
      # disable radar ECU by setting to programming mode, see RADAR_DISABLE_FRAME
      elif CS.radar_alive or not self.radar_disable_sent:
        # a knockout can get lost, and the radar comes back if its session ever ends: ask again
        # every second for as long as it is on the bus
        if self.frame >= RADAR_DISABLE_FRAME and (not self.radar_disable_sent or self.frame % 100 == 0):
          can_sends.append(create_disable_radar())
          self.radar_disable_sent = True
        # only emulate while the real radar is quiet, two ECUs transmitting 0x2B6 at once
        # would collide on the bus
        self.radar_disabled = self.radar_disable_sent and not CS.radar_alive
      else:
        self.radar_disabled = True
        if self.frame % 100 == 0:
          # keep radar ECU disabled by sending tester present
          can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))

      # Highest torque seen without gas input: ~1000
      # Lowest torque seen without break mode: -560 (but only when transitioning from brake to accel mode, else -248)
      # Lowest brake mode accel seen: -4.85m/s²

      # The PCM keeps cruiseState.enabled up for ~80 ms after the driver takes over, but
      # controlsd zeroes actuators.accel the moment longActive drops. Keying the emulation
      # off the PCM alone made those frames claim an active ACC with a *positive* torque
      # request while the car was still decelerating, and the ESP (UC_FREIN) latched
      # ACC_ETAT_DECEL_OR_ESP_STATUS = 3 (accFaulted) within 30 ms of the inversion.
      # Measured with the driver braking over a -1.25 m/s² command. Both signals have to
      # come off the same clock.
      # A gas press is an override, not a disengage: longActive drops but cruise stays on,
      # and the stock radar keeps ACC advertised as suspended (ACC_STATUS 5) instead of
      # flipping it off and back on. brakePressed keeps the inversion guard above.
      gas_override = CS.out.gasPressed and not CS.out.brakePressed
      long_enabled = CS.out.cruiseState.enabled and (CC.longActive or gas_override)

      decel_active = braking and long_enabled

      # Standstill hold / drive-away, mirroring the stock radar sequence (see DRIVE_AWAY_FRAMES).
      # Driven off the planner's own request rather than the pitch-compensated one: on a
      # descent the gravity term alone keeps accel_cmd negative, and the hold would never
      # release however hard the planner asked to move.
      # A gas press ends the hold immediately. The stock radar drops the decel request and
      # advertises ACC suspended when the driver takes over; holding the ESP instead put a
      # decel request up against the driver's own torque and latched the fault
      # (ACC_ETAT_DECEL_OR_ESP_STATUS 2 -> 0 -> 3 within ~0.5 s of the pedal).
      if not long_enabled or CS.out.gasPressed:
        self.hold = False
        self.launching = False
        self.drive_away_frames = 0
      elif self.hold:
        if actuators.accel > 0.0:
          self.hold = False
          self.drive_away_frames = DRIVE_AWAY_FRAMES
      elif self.drive_away_frames > 0:
        self.drive_away_frames -= 1
        if self.drive_away_frames == 0:
          self.launching = True
      elif self.launching:
        if CS.out.vEgo >= LAUNCH_COMPLETE_SPEED:
          self.launching = False
        elif CS.out.standstill and actuators.accel <= 0.0:  # lead stopped again before we got rolling
          self.launching = False
          self.hold = True
      elif CS.out.standstill and actuators.accel <= 0.0:
        self.hold = True
      drive_away = self.drive_away_frames > 0
      standstill_hold = self.hold or drive_away

      # an EV motor answers a torque step within a frame, so the request is rate limited while it
      # is the torque that drives the car; the launch keeps its step, the ESP is still in control
      self.long_frames = self.long_frames + 1 if CC.longActive else 0
      torque_driving = CC.longActive and not decel_active and not standstill_hold and not self.launching
      if self.ev and long_enabled and not decel_active and not standstill_hold:
        if torque_driving and self.torque_last is not None:
          torque = min(max(torque, self.torque_last - P.EV_TORQUE_RATE), self.torque_last + P.EV_TORQUE_RATE)
        if self.long_frames < P.EV_LIFT_OFF_FRAMES:
          torque = max(torque, P.EV_LIFT_OFF_TORQUE_MIN)
      self.torque_last = torque if torque_driving else None

      # stand in for the radar for exactly as long as it is off the bus
      if self.radar_disabled and not self.radar_released and self.frame % 2 == 0:
        can_sends.append(create_HS2_DYN1_MDD_ETAT_2B6(self.packer, self.frame // 2, accel, decel_active, long_enabled,
                                                      CS.out.gasPressed, CS.out.brakePressed, CS.out.standstill, torque,
                                                      standstill_hold, self.launching))
        can_sends.append(create_HS2_DYN_MDD_ETAT_2F6(self.packer, decel_active or standstill_hold or self.launching,
                                                     CC.hudControl.leadVisible, self.bars, drive_away))

    # stock long
    # emulate resume button every 3 seconds to prevent autohold timeout
    elif CC.latActive and CS.out.standstill and CC.hudControl.leadVisible:
      # map: {frame:status} - 0, 1
      status = {0: 0, 5: 1}.get(self.frame % 300)
      if status is not None:
        msg = CS.hs2_dat_mdd_cmd_452
        counter = (msg['COUNTER'] + 1) % 16
        can_sends.append(create_resume_acc(self.packer, counter, status, msg))

    can_sends.append(create_lka_steering(self.packer, lat_active, apply_angle, self.status, CS.lka_drive_mode))
    self.apply_angle_last = apply_angle

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1
    return new_actuators, can_sends
