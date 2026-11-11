import math
import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm, get_max_angle_vm
from opendbc.car.psa.psacan import (create_lka_steering, create_resume_acc, create_disable_radar, create_enable_radar,
                                    create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6)
from opendbc.car.psa.values import CarControllerParams, PSAFlags
from opendbc.car.vehicle_model import VehicleModel

try:
  from openpilot.cereal import messaging
  sm = messaging.SubMaster(['modelV2'], poll='modelV2')
except ImportError:
  # cereal is only available in openpilot, not in standalone opendbc
  sm = None

# the panda rejects the radar emulation until it is in the PSA safety mode, wait for that before the knockout
RADAR_DISABLE_FRAME = 100  # 1.0 s
# bounds the radar release when the real 0x2B6 never comes back
RADAR_ENABLE_TIMEOUT_FRAMES = 200  # 2.0 s

ENGAGE_FRAMES = 100  # 1.0 s blend from the wheel angle to the command when lateral engages
DRIVE_AWAY_FRAMES = 4  # 40 ms DRIVE_AWAY_REQUEST pulse, as the stock radar sends
LAUNCH_COMPLETE_SPEED = 0.5  # m/s, decel request released above this

# wheel torque request for the pitch compensated accel, drag and the launch are left to the long integrator
ACCEL_LOOKUP = [-1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 3.0]         # m/s²
TORQUE_LOOKUP = [-400, -140, 82, 375, 615, 865, 1100, 1600]       # N.m, 208 and Corsa F
TORQUE_LOOKUP_EV = [-400, -130, 170, 350, 645, 862, 1100, 1534]   # N.m, e-208


def get_safety_CP():
  # We use the PSA_PEUGEOT_208 platform for lateral limiting to match safety
  from opendbc.car.psa.interface import CarInterface
  return CarInterface.get_non_essential_params("PSA_PEUGEOT_208")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.VM = VehicleModel(get_safety_CP())
    self.ev = bool(CP.flags & PSAFlags.EV)

    # lateral
    self.apply_angle_last = 0
    self.torque_factor = 0
    self.lat_active_last = False
    self.engage_frame = 0
    self.status = 2

    # radar knockout and release
    self.radar_disable_sent = False
    self.radar_disabled = False
    self.radar_release_requested = False
    self.radar_released = False
    self.radar_release_frame = 0

    # longitudinal
    self.bars = 4
    self.hold = False
    self.launching = False
    self.drive_away_frames = 0
    self.braking = False
    self.pitch = FirstOrderFilter(0., 1.0, DT_CTRL)
    self.torque_last = None
    self.long_frames = 0

  def release_radar(self) -> bool:
    """Hand the ADAS bus back to the radar ECU. Non-blocking, returns True once it is back.

    Has to run while still onroad, the panda rejects everything once it is offroad.
    """
    if not self.CP.openpilotLongitudinalControl:
      return True
    self.radar_release_requested = True
    return self.radar_released

  def update(self, CC, CS, now_nanos):
    P = CarControllerParams
    can_sends = []
    actuators = CC.actuators

    # *** lateral ***
    apply_angle = actuators.steeringAngleDeg

    # don't take over a wheel that is beyond the lateral accel limit, safety blocks the unwind
    max_angle = get_max_angle_vm(max(CS.out.vEgoRaw, 1), self.VM, P)
    lat_active = CC.latActive and (self.lat_active_last or abs(CS.out.steeringAngleDeg) < max_angle)

    # blend from the wheel angle to the command on engage
    if lat_active and not self.lat_active_last:
      self.engage_frame = 0
    if lat_active and self.engage_frame < ENGAGE_FRAMES:
      self.engage_frame += 1
      blend = self.engage_frame / ENGAGE_FRAMES
      apply_angle = blend * apply_angle + (1 - blend) * CS.out.steeringAngleDeg
    self.lat_active_last = lat_active

    # low-pass filter the command at low speed to suppress jitter
    if lat_active and CS.out.vEgoRaw < 5.0:
      tau = np.interp(CS.out.vEgoRaw, [0.5, 5.0], [0.3, 0.1])
      alpha = 1 - math.exp(-DT_CTRL / tau)
      apply_angle = alpha * apply_angle + (1 - alpha) * self.apply_angle_last

    apply_angle = apply_steer_angle_limits_vm(apply_angle, self.apply_angle_last, CS.out.vEgoRaw,
                                              CS.out.steeringAngleDeg, lat_active, P, self.VM)

    # less EPS assist the harder the driver pushes on the wheel, wound down to zero when lateral goes inactive
    if lat_active:
      target = int(np.interp(abs(CS.out.steeringTorque), P.TORQUE_FACTOR_DRIVER_BP, [P.TORQUE_FACTOR_MAX, P.TORQUE_FACTOR_MIN]))
      self.torque_factor = min(max(target, self.torque_factor - P.TORQUE_FACTOR_STEP_DOWN), self.torque_factor + P.TORQUE_FACTOR_STEP_UP)
    else:
      self.torque_factor = max(self.torque_factor - P.TORQUE_FACTOR_STEP_DOWN, 0)

    # the EPS drops out on driver override, re-engage with READY -> AUTHORIZED -> ACTIVE
    # STATUS - 0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
    if self.torque_factor == 0:
      self.status = 2
    elif CS.eps_state in (1, 3):  # authorized, active
      self.status = 4
    elif self.frame % P.STATUS_STEP == 0:
      self.status = 2 if self.status == 4 else self.status + 1

    # *** longitudinal ***
    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])
    accel_slope = math.sin(self.pitch.x) * 9.81

    accel = min(max(actuators.accel, P.ACCEL_MIN), P.ACCEL_MAX)
    accel_cmd = accel + accel_slope

    # hysteresis between the torque request and the friction brakes
    brake_on, brake_off = (P.EV_BRAKE_ON_ACCEL, P.EV_BRAKE_OFF_ACCEL) if self.ev else (P.BRAKE_ON_ACCEL, P.BRAKE_OFF_ACCEL)
    brake_accel = brake_off if self.braking else brake_on

    torque_nm = np.interp(accel_cmd, ACCEL_LOOKUP, TORQUE_LOOKUP_EV if self.ev else TORQUE_LOOKUP)
    torque = float(max(P.TORQUE_MIN, min(torque_nm, P.TORQUE_MAX)))

    braking = accel_cmd < brake_accel and not CS.out.gasPressed
    self.braking = braking

    if self.CP.openpilotLongitudinalControl:
      # lead distance bars
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
            self.radar_released = True
          else:
            if self.radar_release_frame == 0:
              can_sends.append(create_enable_radar())
            self.radar_release_frame += 1
            self.radar_released = CS.radar_alive or self.radar_release_frame >= RADAR_ENABLE_TIMEOUT_FRAMES
      elif CS.radar_alive or not self.radar_disable_sent:
        # disable the radar ECU, asked again every second for as long as it is on the bus
        if self.frame >= RADAR_DISABLE_FRAME and (not self.radar_disable_sent or self.frame % 100 == 0):
          can_sends.append(create_disable_radar())
          self.radar_disable_sent = True
        # only emulate while the real radar is quiet
        self.radar_disabled = self.radar_disable_sent and not CS.radar_alive
      else:
        self.radar_disabled = True
        if self.frame % 100 == 0:
          # keep the radar ECU disabled
          can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))

      # a gas press is an override, ACC stays advertised. A brake press must drop it in the same frame as
      # longActive, the ESP faults on an active ACC with a positive torque request while the car decelerates
      gas_override = CS.out.gasPressed and not CS.out.brakePressed
      long_enabled = CS.out.cruiseState.enabled and (CC.longActive or gas_override)

      decel_active = braking and long_enabled

      # standstill hold -> drive away pulse -> launch, as the stock radar does. Off the planner's request
      # and not the pitch compensated one, or the hold never releases on a descent
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

      # rate limit the torque request of an EV while it drives the car, and keep it out of the lift-off regen
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

    # stock long: resume button every 3 seconds to prevent the autohold timeout
    elif CC.latActive and CS.out.standstill and CC.hudControl.leadVisible:
      status = {0: 0, 5: 1}.get(self.frame % 300)
      if status is not None:
        msg = CS.hs2_dat_mdd_cmd_452
        counter = (msg['COUNTER'] + 1) % 16
        can_sends.append(create_resume_acc(self.packer, counter, status, msg))

    can_sends.append(create_lka_steering(self.packer, self.torque_factor, apply_angle, self.status, CS.lka_drive_mode))
    self.apply_angle_last = apply_angle

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1
    return new_actuators, can_sends
