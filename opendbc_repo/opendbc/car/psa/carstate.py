import copy
from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.values import DBC, CarControllerParams, EpsState, PSAFlags
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType

# Radar ECU (ARTIV) message used to tell whether the stock radar is still on the bus.
RADAR_MSG = 'HS2_DYN1_MDD_ETAT_2B6'
# 0x2B6 is 50 Hz and update() runs at 100 Hz, so this counts 10 ms per frame and the
# whole of it lands inside the ADAS bus silence the knockout opens up: the emulation
# only starts once this has expired. The ESP (UC_FREIN) marks its ACC fields invalid
# after ~150 ms without 0x2B6, so this has to be a fraction of that budget, not all
# of it. 15 frames (150 ms) spent the budget exactly and faulted the car — the ESP
# flagged 152 ms after the radar's last frame, 21 ms before the emulation's first.
# Bound below by the message's own jitter: 20.2 ms
# median, 35.4 ms worst over 4932 frames across 4 routes. 6 frames is 1.7x that worst
# gap, and holds the total silence to ~70 ms.
RADAR_TIMEOUT_FRAMES = 6

# the CMM briefly drops the ACC torque when switching to the brakes
ACC_TORQUE_LOST_FRAMES = 100

# SPEED_SETPOINT reads 2 km/h below the set speed shown on the dash — driver-observed, and the
# same at every set speed. The car regulates to the value on the bus, absorbing the speedometer's
# over-read, so that value stays the control target and only the cluster display is corrected;
# without this openpilot's own set speed sits a step below the dash's for the whole drive.
CLUSTER_SETPOINT_OFFSET = 2  # km/h

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.radar_alive = False
    self.radar_last_ts = 0
    self.radar_stale_frames = 0
    self.acc_torque_lost_frames = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    cp_adas = can_parsers[Bus.adas]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'],
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD
    ret.standstill = bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL'])

    # gas
    ret.gasPressed = cp_cam.vl['DRIVER']['GAS_PEDAL'] > 0

    # brake, the ESP also sees light presses that the BSI brake switch (Dat_BSI P013_MainBrake) misses
    ret.brakePressed = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['APPUI_FREIN_RECONSTRUIT'] == 2
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1 # 0: disengaged, 1: engaged, 3: brake actuator moving

    # steering wheel
    cp_eps = cp_cam if self.CP.flags & PSAFlags.EPS_CAM_BUS else cp
    ret.steeringAngleDeg = cp_eps.vl['STEERING_ALT']['ANGLE']  # EPS
    ret.steeringRateDeg = cp_eps.vl['STEERING_ALT']['RATE'] * (1 - 2 * cp_eps.vl['STEERING_ALT']['RATE_SIGN'])  # RATE_SIGN [0,1] to [1,-1]
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    self.eps_state = cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA']
    self.eps_active = self.eps_state == EpsState.ACTIVE
    ret.steerFaultTemporary = self.eps_state == EpsState.DEFECT

    # cruise
    setpoint = cp_adas.vl['HS2_DAT_MDD_CMD_452']['SPEED_SETPOINT']  # set to 255 when ACC is off
    # the setpoint is the wheel-speed value the stock system regulates to; vEgo is wheel speed
    # * wheelSpeedFactor, so scale the target the same way or cruise settles that factor below
    # the speed stock ACC would hold for the same setpoint
    ret.cruiseState.speed = setpoint * CV.KPH_TO_MS * self.CP.wheelSpeedFactor
    # show what the dash shows, see CLUSTER_SETPOINT_OFFSET; with ACC off there is nothing to offset
    ret.cruiseState.speedCluster = (setpoint + CLUSTER_SETPOINT_OFFSET) * CV.KPH_TO_MS if setpoint < 255 else ret.cruiseState.speed
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['RVV_ACC_ACTIVATION_REQ'] == 1
    # the stock radar reports ACC_STATUS 2 (inhibited) on every brake press, so it cannot stand in for available
    ret.cruiseState.available = True
    # 0: None, 1: CC, 2: Limiter, 3: ACC. None is just ACC switched off, only plain cruise and the limiter are the wrong mode
    ret.cruiseState.nonAdaptive = cp_adas.vl['HS2_DAT_MDD_CMD_452']['LONGITUDINAL_REGULATION_TYPE'] in (1, 2)
    # with openpilot long the hold is ours, see the standstill sequence in carcontroller.py
    ret.cruiseState.standstill = not self.CP.openpilotLongitudinalControl and bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL'])
    ret.accFaulted = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['ACC_ETAT_DECEL_OR_ESP_STATUS'] == 3
    # resume request
    self.hs2_dat_mdd_cmd_452 = copy.copy(cp_adas.vl['HS2_DAT_MDD_CMD_452'])

    # Is the stock radar ECU still transmitting? Our own emulated 0x2B6 comes back
    # as a TX echo on src 129, which this parser drops, so this only ever tracks the
    # real ECU. Compare timestamps rather than clocks so this stays replay-safe.
    radar_ts = cp_adas.ts_nanos[RADAR_MSG]['COUNTER']
    if radar_ts != self.radar_last_ts:
      self.radar_last_ts = radar_ts
      self.radar_stale_frames = 0
    else:
      self.radar_stale_frames += 1
    self.radar_alive = radar_ts != 0 and self.radar_stale_frames < RADAR_TIMEOUT_FRAMES

    # The CMM can stop taking our torque for the rest of the drive. It drops ACC_GO_RELATED when it
    # does, and its own pedal reading shows a gas press before DRIVER does. Under the emulation the
    # flag was never down without gas for longer than 0.12 s, measured over four engaged drives
    # including standstill holds, so a second of it is the CMM having let go.
    gas_cmm = cp.vl['Dyn_CMM']['P002_Com_rAPP'] > 0
    acc_torque_lost = (self.CP.openpilotLongitudinalControl and not self.radar_alive and ret.cruiseState.enabled and
                       not (ret.gasPressed or gas_cmm) and cp_adas.vl['NEW_MSG_318']['ACC_GO_RELATED'] == 0)
    self.acc_torque_lost_frames = self.acc_torque_lost_frames + 1 if acc_torque_lost else 0
    ret.accFaulted = ret.accFaulted or self.acc_torque_lost_frames > ACC_TORQUE_LOST_FRAMES

    # gear
    if bool(cp_cam.vl['Dat_BSI']['P103_Com_bRevGear']):
      ret.gearShifter = GearShifter.reverse
    else:
      # Both D and B are forward-driving gears; always report drive so openpilot stays enabled
      ret.gearShifter = GearShifter.drive

    # Store raw DRIVE signal to echo back on CAN (0: D, 1: B/brake mode)
    self.lka_drive_mode = int(cp_cam.vl['LANE_KEEP_ASSIST']['DRIVE'])

    # blinkers
    blinker = cp_cam.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC']
    ret.leftBlinker = blinker == 1
    ret.rightBlinker = blinker == 2

    # lock info
    ret.doorOpen = any((cp_cam.vl['Dat_BSI']['DRIVER_DOOR'], cp_cam.vl['Dat_BSI']['PASSENGER_DOOR']))
    ret.seatbeltUnlatched = cp_cam.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      # nan frequency: openpilot longitudinal deliberately silences the radar ECU,
      # so a missing 0x2B6 must not invalidate the ADAS bus
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [(RADAR_MSG, float('nan')), ('NEW_MSG_318', float('nan'))], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
