import copy
from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.values import DBC, CarControllerParams, EpsState, PSAFlags
from opendbc.car.interfaces import CarStateBase

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

# stock radar ECU message, 50 Hz. The ESP faults after ~150 ms without it, so the radar counts as gone well before that
RADAR_MSG = 'HS2_DYN1_MDD_ETAT_2B6'
RADAR_TIMEOUT_FRAMES = 6  # 60 ms

# the CMM briefly drops the ACC torque when switching to the brakes
ACC_TORQUE_LOST_FRAMES = 100

# SPEED_SETPOINT reads 2 km/h below the set speed shown on the dash
CLUSTER_SETPOINT_OFFSET = 2  # km/h

# values of SPEED_SETPOINT and INTER_VEHICLE_TIME_SETPOINT with ACC off, 0 before the first frame
SETPOINT_OFF = {0, 255}  # km/h
GAP_OFF = 6.0  # s, the idle value is 6.2
# time gap to openpilot personality: the stalk steps through 2.0, 1.5 and 1.0 s
GAP_RELAXED = 1.75  # s and above
GAP_AGGRESSIVE = 1.25  # s and below


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.radar_alive = False
    self.radar_last_ts = 0
    self.radar_stale_frames = 0
    self.acc_torque_lost_frames = 0
    self.setpoint_last = 0
    self.gap_last = 0.
    self.gap_personality = None  # LongitudinalPersonality the time gap asks for, None with ACC off or stock ACC

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

    # brake, the ESP also sees light presses that the BSI brake switch misses
    ret.brakePressed = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['APPUI_FREIN_RECONSTRUIT'] == 2
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1  # 0: disengaged, 1: engaged, 3: brake actuator moving

    # steering wheel
    cp_eps = cp_cam if self.CP.flags & PSAFlags.EPS_CAM_BUS else cp
    ret.steeringAngleDeg = cp_eps.vl['STEERING_ALT']['ANGLE']
    ret.steeringRateDeg = cp_eps.vl['STEERING_ALT']['RATE'] * (1 - 2 * cp_eps.vl['STEERING_ALT']['RATE_SIGN'])  # RATE_SIGN [0,1] to [1,-1]
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    self.eps_state = cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA']
    self.eps_active = self.eps_state == EpsState.ACTIVE
    ret.steerFaultTemporary = self.eps_state == EpsState.DEFECT

    # cruise
    setpoint = cp_adas.vl['HS2_DAT_MDD_CMD_452']['SPEED_SETPOINT']  # 255 when ACC is off
    # the car regulates wheel speed to the setpoint, scale it like vEgo
    ret.cruiseState.speed = setpoint * CV.KPH_TO_MS * self.CP.wheelSpeedFactor
    ret.cruiseState.speedCluster = (setpoint + CLUSTER_SETPOINT_OFFSET) * CV.KPH_TO_MS if setpoint < 255 else ret.cruiseState.speed
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['RVV_ACC_ACTIVATION_REQ'] == 1
    ret.cruiseState.available = True
    ret.cruiseState.nonAdaptive = cp_adas.vl['HS2_DAT_MDD_CMD_452']['LONGITUDINAL_REGULATION_TYPE'] in (1, 2)  # 0: None, 1: CC, 2: Limiter, 3: ACC
    # with openpilot long the standstill hold is ours
    ret.cruiseState.standstill = not self.CP.openpilotLongitudinalControl and bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL'])
    ret.accFaulted = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['ACC_ETAT_DECEL_OR_ESP_STATUS'] == 3
    # resume request
    self.hs2_dat_mdd_cmd_452 = copy.copy(cp_adas.vl['HS2_DAT_MDD_CMD_452'])

    # the stalk buttons are not on CAN, the BSI only sends what they change: a short press per change
    gap = cp_adas.vl['HS2_DAT_MDD_CMD_452']['INTER_VEHICLE_TIME_SETPOINT']
    button = None
    # with openpilot longitudinal the car's time gap is the master: card writes this to LongitudinalPersonality.
    # Not a distance press, StarPilot drops those for PSA before they reach the personality
    self.gap_personality = None
    if self.CP.openpilotLongitudinalControl and 0 < gap < GAP_OFF:
      self.gap_personality = 2 if gap >= GAP_RELAXED else 0 if gap <= GAP_AGGRESSIVE else 1
    if setpoint != self.setpoint_last and not {setpoint, self.setpoint_last} & SETPOINT_OFF:
      button = ButtonType.accelCruise if setpoint > self.setpoint_last else ButtonType.decelCruise
    elif not self.CP.openpilotLongitudinalControl and gap != self.gap_last and 0 < min(gap, self.gap_last) and max(gap, self.gap_last) < GAP_OFF:
      button = ButtonType.gapAdjustCruise
    if button is not None:
      ret.buttonEvents = [structs.CarState.ButtonEvent(pressed=pressed, type=button) for pressed in (True, False)]
    self.setpoint_last, self.gap_last = setpoint, gap

    # is the stock radar ECU still transmitting? Our own 0x2B6 is a TX echo, which the parser drops
    radar_ts = cp_adas.ts_nanos[RADAR_MSG]['COUNTER']
    if radar_ts != self.radar_last_ts:
      self.radar_last_ts = radar_ts
      self.radar_stale_frames = 0
    else:
      self.radar_stale_frames += 1
    self.radar_alive = radar_ts != 0 and self.radar_stale_frames < RADAR_TIMEOUT_FRAMES

    # the CMM drops ACC_GO_RELATED when it stops taking our torque
    gas_cmm = cp.vl['Dyn_CMM']['P002_Com_rAPP'] > 0
    acc_torque_lost = (self.CP.openpilotLongitudinalControl and not self.radar_alive and ret.cruiseState.enabled and
                       not (ret.gasPressed or gas_cmm) and cp_adas.vl['NEW_MSG_318']['ACC_GO_RELATED'] == 0)
    self.acc_torque_lost_frames = self.acc_torque_lost_frames + 1 if acc_torque_lost else 0
    ret.accFaulted = ret.accFaulted or self.acc_torque_lost_frames > ACC_TORQUE_LOST_FRAMES

    # gear, D and B both report drive
    if bool(cp_cam.vl['Dat_BSI']['P103_Com_bRevGear']):
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = GearShifter.drive

    # raw DRIVE signal to echo back on CAN (0: D, 1: B)
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
      # nan frequency: openpilot longitudinal silences the radar ECU, a missing 0x2B6 must not invalidate the bus
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [(RADAR_MSG, float('nan')), ('NEW_MSG_318', float('nan'))], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
