from opendbc.car.can_definitions import CanData

RADAR_DIAG_ADDR = 0x6B6


def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x32D: 0xD, 0x42D: 0xC, 0x2B6: 0xC, 0x2F6: 0x8}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF


def create_lka_steering(packer, torque_factor: int, apply_angle: float, status: int, lka_drive_mode: int):
  values = {
    'DRIVE': lka_drive_mode,  # 0: D, 1: B, echoed from the car
    'STATUS': status,
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': torque_factor,  # percent of EPS assist, 0 is inactive
    'SET_ANGLE': apply_angle,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


def create_resume_acc(packer, counter, status, hs2_dat_mdd_cmd_452):
  hs2_dat_mdd_cmd_452['COUNTER'] = counter
  hs2_dat_mdd_cmd_452['COCKPIT_GO_ACC_REQUEST'] = status
  return packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, hs2_dat_mdd_cmd_452)


# Radar, 50 Hz
def create_HS2_DYN1_MDD_ETAT_2B6(packer, frame: int, accel: float, decel_active: bool, enabled: bool,
                                 gasPressed: bool, brakePressed: bool, standstill: bool, torque: int,
                                 standstill_hold: bool = False, launching: bool = False):
  # Stock standstill sequence:
  #   hold:   decel request with the -10.65 hold code, potential torque only
  #   launch: desired decel steps to +2.0 with the decel request still up, this is what the ESP releases on
  # ACC_STATUS stays active (4) for as long as a decel request is up, the release has to be atomic
  torque_mode = enabled and not decel_active and not standstill_hold
  decel_req = decel_active or standstill_hold or launching
  values = {
    'MDD_DESIRED_DECELERATION': -10.65 if standstill_hold else 2.0 if launching else accel if decel_active else 2.05,  # m/s², 2.05 is idle
    'POTENTIAL_WHEEL_TORQUE_REQUEST': 2 if decel_active and not standstill_hold else (1 if enabled else 0),
    'MIN_TIME_FOR_DESIRED_GEAR': 6.2 if torque_mode or standstill_hold else 0.0,
    'GMP_POTENTIAL_WHEEL_TORQUE': torque if torque_mode or standstill_hold else -4000,
    'ACC_STATUS': 4 if decel_req else ((5 if gasPressed else 2 if brakePressed and not standstill else 4) if enabled else (2 if brakePressed else 3)),
    'GMP_WHEEL_TORQUE': torque if torque_mode else -4000,
    'WHEEL_TORQUE_REQUEST': 1 if torque_mode else 0,
    'AUTO_BRAKING_STATUS': 3,  # AEB
    'MDD_DECEL_TYPE': 1 if decel_req else 0,
    'MDD_DECEL_CONTROL_REQ': 1 if decel_req else 0,
  }

  return packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, values)


# Radar, 50 Hz
def create_HS2_DYN_MDD_ETAT_2F6(packer, decel_active: bool, lead_visible: bool, lead_distance_bars: int,
                                drive_away: bool = False):
  values = {
    'DRIVE_AWAY_REQUEST': drive_away,
    'TARGET_DETECTED': lead_visible,
    'DISPLAY_INTERVEHICLE_TIME': 5.0,
    'MDD_DECEL_CONTROL_REQ': decel_active,
    'TARGET_POSITION': lead_distance_bars,  # distance to the lead, far - 4, 3, 2, 1 - near
  }

  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, values)


def create_disable_radar():
  # programming session, the radar ECU stops broadcasting
  return CanData(RADAR_DIAG_ADDR, bytes([0x02, 0x10, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00]), 1)


def create_enable_radar():
  # back to the default session, the radar ECU resumes broadcasting
  return CanData(RADAR_DIAG_ADDR, bytes([0x02, 0x10, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00]), 1)
