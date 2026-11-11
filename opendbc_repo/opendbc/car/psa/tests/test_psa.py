import unittest

import numpy as np

from opendbc.can import CANParser, CANPacker
from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.carcontroller import RADAR_DISABLE_FRAME, RADAR_ENABLE_TIMEOUT_FRAMES
from opendbc.car.psa.carstate import ACC_TORQUE_LOST_FRAMES, CLUSTER_SETPOINT_OFFSET, RADAR_TIMEOUT_FRAMES
from opendbc.car.psa.values import CarControllerParams, PSAFlags

LongCtrlState = structs.CarControl.Actuators.LongControlState
P = CarControllerParams

DISABLE_RADAR = (0x6B6, b'\x02\x10\x02\x80\x00\x00\x00\x00')
ENABLE_RADAR = (0x6B6, b'\x02\x10\x01\x80\x00\x00\x00\x00')
RADAR_EMULATION = (0x2B6, 0x2F6)
PSA_ADAS_BUS = 1


def make_car_interface(alpha_long: bool, ev: bool = False):
  CarInterface = interfaces["PSA_PEUGEOT_208"]
  fingerprint = {i: {} for i in range(8)}
  if ev:
    fingerprint[PSA_ADAS_BUS][0x202] = 8
  CP = CarInterface.get_params("PSA_PEUGEOT_208", fingerprint, [],
                               alpha_long=alpha_long, is_release=False, docs=False)
  return CarInterface(CP)


class TestPsaRadarKnockout(unittest.TestCase):
  """The radar ECU knockout has to stay in step with what is actually on the ADAS bus.

  The ESP (UC_FREIN) marks its ACC fields invalid after ~150 ms without 0x2B6, which
  openpilot reports as accFaulted, so 0x2B6 must never be missing from the bus for
  longer than that and must never be sent by two ECUs at once.
  """

  def setUp(self):
    self.CI = make_car_interface(alpha_long=True)
    self.assertTrue(self.CI.CP.openpilotLongitudinalControl)
    self.CC = structs.CarControl().as_reader()
    self.now_nanos = 0

  def step(self, radar_alive: bool):
    self.CI.update([])
    self.CI.CS.radar_alive = radar_alive
    _, can_sends = self.CI.apply(self.CC, self.now_nanos)
    self.now_nanos += int(DT_CTRL * 1e9)
    return can_sends

  @staticmethod
  def addrs(can_sends):
    return [msg[0] for msg in can_sends]

  def test_no_knockout_before_safety_mode_is_certain(self):
    # pandad still has the panda in ELM327 mode early on, where the diagnostic
    # knockout is allowed through but the emulation that replaces it is not
    for _ in range(RADAR_DISABLE_FRAME):
      addrs = self.addrs(self.step(radar_alive=True))
      self.assertNotIn(0x6B6, addrs)
      for addr in RADAR_EMULATION:
        self.assertNotIn(addr, addrs)

  def test_knockout_sent_once_then_waits_for_radar_to_go_quiet(self):
    for _ in range(RADAR_DISABLE_FRAME):
      self.step(radar_alive=True)

    # the knockout goes out exactly once
    assert DISABLE_RADAR in [(m[0], m[1]) for m in self.step(radar_alive=True)]

    # the real radar is still transmitting, so we must not emulate yet
    for _ in range(50):
      addrs = self.addrs(self.step(radar_alive=True))
      self.assertNotIn(0x6B6, addrs)
      for addr in RADAR_EMULATION:
        self.assertNotIn(addr, addrs, "emulating while the real radar is still on the bus")

    # once it goes quiet, emulation takes over
    seen: set[int] = set()
    for _ in range(4):
      seen.update(self.addrs(self.step(radar_alive=False)))
    self.assertTrue(RADAR_EMULATION[0] in seen and RADAR_EMULATION[1] in seen)

  def test_knockout_is_repeated_while_the_radar_stays_on_the_bus(self):
    sent = [DISABLE_RADAR in [(m[0], m[1]) for m in self.step(radar_alive=True)] for _ in range(RADAR_DISABLE_FRAME + 250)]
    self.assertEqual([i for i, s in enumerate(sent) if s], [100, 200, 300])

  def test_radar_coming_back_stops_the_emulation_and_is_knocked_out_again(self):
    for _ in range(RADAR_DISABLE_FRAME + 1):
      self.step(radar_alive=True)
    for _ in range(10):
      self.step(radar_alive=False)
    self.assertTrue(self.CI.CC.radar_disabled)

    sent = []
    for _ in range(100):
      sends = self.step(radar_alive=True)
      sent += [(m[0], m[1]) for m in sends]
      for addr in RADAR_EMULATION:
        self.assertNotIn(addr, self.addrs(sends), "emulating over a radar that came back")
    self.assertIn(DISABLE_RADAR, sent)

  def test_no_knockout_without_openpilot_longitudinal(self):
    self.CI = make_car_interface(alpha_long=False)
    self.assertFalse(self.CI.CP.openpilotLongitudinalControl)
    for _ in range(RADAR_DISABLE_FRAME * 2):
      addrs = self.addrs(self.step(radar_alive=True))
      self.assertNotIn(0x6B6, addrs)
      for addr in RADAR_EMULATION:
        self.assertNotIn(addr, addrs)


class TestPsaRadarHandoverBudget(unittest.TestCase):
  """The knockout hands 0x2B6 from the radar to openpilot, and the gap is on a clock.

  TestPsaRadarKnockout injects radar_alive directly, so it says nothing about how long
  that flag takes to turn over. That latency is the gap: the emulation does not start
  until CarState has counted RADAR_TIMEOUT_FRAMES of missing 0x2B6. A drive that spent
  the whole ESP budget there faulted the car.
  """

  # UC_FREIN marks its ACC fields invalid this long after the last 0x2B6. Measured
  # three ways in the 2026-08-15 logs: 150 ms, 160 ms, and a 152 ms fault.
  ESP_TOLERANCE = 0.150
  # worst 0x2B6 inter-frame gap over 4932 steady-state frames across 4 routes; the
  # median is 20.2 ms, so anything below this reads a live radar as a dead one
  WORST_RADAR_JITTER = 0.0354

  def silence(self):
    """Longest ADAS bus silence the knockout can open up, worst case."""
    # the radar's last frame can land a full period before the knockout goes out
    radar_period = 0.020
    # CarState needs this many 10 ms frames of no 0x2B6 to call the radar dead
    detection = RADAR_TIMEOUT_FRAMES * DT_CTRL
    # and the emulation is gated to even frames, so it can wait one more
    return radar_period + detection + DT_CTRL

  def test_emulation_starts_before_the_esp_gives_up(self):
    self.assertLess(self.silence(), self.ESP_TOLERANCE,
                    "0x2B6 goes missing for longer than the ESP tolerates; the car faults with accFaulted as alpha long engages")

  def test_emulation_starts_with_margin(self):
    # 150 ms is where it faulted, not where it is safe. The jitter floor below puts a
    # hard lower bound on the detection half of the budget, so 1.5x is what is on offer.
    self.assertLess(self.silence() * 1.5, self.ESP_TOLERANCE,
                    "no useful margin on the ESP's 150 ms tolerance")

  def test_detection_is_slower_than_the_radar_jitter(self):
    # too short and a live radar reads as dead, putting two ECUs on 0x2B6 at once
    self.assertGreater(RADAR_TIMEOUT_FRAMES * DT_CTRL, self.WORST_RADAR_JITTER * 1.5,
                       "would call the radar dead on normal 0x2B6 jitter")


class TestPsaRadarRelease(unittest.TestCase):
  """Switching alpha long off has to close the gap between us leaving 0x2B6 and the radar returning.

  This runs inside the control loop, on purpose: pandad puts the panda in NO_OUTPUT the
  moment deviceState goes offroad, so anything sent from the shutdown path is rejected and
  the radar only comes back on its own S3 timeout, ~5 s later. Measured at 4.42 s of
  silence, with the ESP latched for the whole of the next drive.
  """

  def setUp(self):
    self.CI = make_car_interface(alpha_long=True)
    self.CC = structs.CarControl().as_reader()
    self.now_nanos = 0

  def step(self, radar_alive: bool):
    self.CI.update([])
    self.CI.CS.radar_alive = radar_alive
    _, can_sends = self.CI.apply(self.CC, self.now_nanos)
    self.now_nanos += int(DT_CTRL * 1e9)
    return can_sends

  def knock_out(self):
    """Run up to a steady state with the radar off the bus and openpilot emulating it."""
    for _ in range(RADAR_DISABLE_FRAME + 1):
      self.step(radar_alive=True)
    for _ in range(10):
      self.step(radar_alive=False)
    assert self.CI.CC.radar_disabled

  @staticmethod
  def addrs(can_sends):
    return [msg[0] for msg in can_sends]

  def test_asks_the_radar_back_and_covers_the_gap(self):
    self.knock_out()

    self.assertFalse(self.CI.release_ecus(), "claimed to be done before touching the radar")
    sent = [(m[0], m[1]) for m in self.step(radar_alive=False)]
    self.assertIn(ENABLE_RADAR, sent, "did not ask the radar back")

    # keep standing in for it until it is transmitting again, and only ask once
    seen: set[int] = set()
    for _ in range(50):
      msgs = self.step(radar_alive=False)
      self.assertNotIn(ENABLE_RADAR, [(m[0], m[1]) for m in msgs])
      seen.update(self.addrs(msgs))
      self.assertFalse(self.CI.release_ecus())
    self.assertTrue(RADAR_EMULATION[0] in seen and RADAR_EMULATION[1] in seen, "left the bus silent while the radar restarted")

  def test_stops_emulating_as_soon_as_the_real_radar_transmits(self):
    self.knock_out()
    self.CI.release_ecus()
    for _ in range(10):
      self.step(radar_alive=False)

    # two ECUs on 0x2B6 would collide, so get out of the way on the first live frame
    self.step(radar_alive=True)
    self.assertTrue(self.CI.release_ecus(), "did not notice the radar came back")
    for _ in range(20):
      addrs = self.addrs(self.step(radar_alive=True))
      self.assertNotIn(0x6B6, addrs)
      for addr in RADAR_EMULATION:
        self.assertNotIn(addr, addrs, "still emulating after the radar returned")

  def test_tester_present_stops_so_the_session_can_end(self):
    self.knock_out()
    self.CI.release_ecus()
    for _ in range(300):
      self.assertNotIn(0x6B6, self.addrs(self.step(radar_alive=False))[1:], "kept the radar in its programming session")

  def test_terminates_if_the_radar_never_comes_back(self):
    self.knock_out()
    self.CI.release_ecus()
    for _ in range(RADAR_ENABLE_TIMEOUT_FRAMES - 1):
      self.step(radar_alive=False)
    self.assertFalse(self.CI.release_ecus())
    self.step(radar_alive=False)
    self.assertTrue(self.CI.release_ecus(), "would hold the onroad cycle forever")

  def test_nothing_to_hand_back_before_the_knockout(self):
    # toggled off inside the RADAR_DISABLE_FRAME window, we never touched the radar
    self.step(radar_alive=True)
    self.CI.release_ecus()
    sent = [(m[0], m[1]) for m in self.step(radar_alive=True)]
    self.assertTrue(self.CI.release_ecus())
    self.assertNotIn(ENABLE_RADAR, sent)

  def test_knockout_never_fires_after_a_release(self):
    self.knock_out()
    self.CI.release_ecus()
    self.step(radar_alive=True)
    for _ in range(RADAR_DISABLE_FRAME * 2):
      sent = [(m[0], m[1]) for m in self.step(radar_alive=True)]
      self.assertNotIn(DISABLE_RADAR, sent, "knocked the radar out again after handing it back")

  def test_release_is_a_noop_without_openpilot_longitudinal(self):
    self.CI = make_car_interface(alpha_long=False)
    self.assertTrue(self.CI.release_ecus())
    for _ in range(RADAR_DISABLE_FRAME * 2):
      self.assertNotIn(0x6B6, self.addrs(self.step(radar_alive=True)))


class PsaEmulationTest(unittest.TestCase):
  """Harness for driving the radar emulation frame by frame and decoding what went out."""

  EV = False

  def setUp(self):
    self.CI = make_car_interface(alpha_long=True, ev=self.EV)
    self.now_nanos = 0
    self.parser = CANParser('psa_aee2010_r3', [('HS2_DYN1_MDD_ETAT_2B6', 0)], PSA_ADAS_BUS)
    self.parser_2f6 = CANParser('psa_aee2010_r3', [('HS2_DYN_MDD_ETAT_2F6', 0)], PSA_ADAS_BUS)

  def step(self, long_active: bool, accel: float, brake_pressed: bool = False, gas_pressed: bool = False, cruise_enabled: bool = True,
           standstill: bool = False, v_ego: float = 0.0, pitch: float = 0.0, lead_visible: bool = False, a_ego: float = 0.0,
           long_state=LongCtrlState.off, cancel: bool = False):
    CC = structs.CarControl()
    CC.longActive = long_active
    CC.cruiseControl.cancel = cancel
    CC.actuators.accel = accel
    CC.actuators.longControlState = long_state
    CC.orientationNED = [0.0, pitch, 0.0]
    CC.hudControl.leadVisible = lead_visible

    self.CI.update([])
    self.CI.CS.radar_alive = False
    self.CI.CS.out.cruiseState.enabled = cruise_enabled
    self.CI.CS.out.brakePressed = brake_pressed
    self.CI.CS.out.gasPressed = gas_pressed
    self.CI.CS.out.standstill = standstill
    self.CI.CS.out.vEgo = v_ego
    self.CI.CS.out.aEgo = a_ego
    _, can_sends = self.CI.apply(CC.as_reader(), self.now_nanos)
    self.now_nanos += int(DT_CTRL * 1e9)
    return can_sends

  def radar_msg(self, can_sends):
    """Decode the emulated 0x2B6 out of a frame's sends, or None if it did not go out."""
    frames = [(m[0], m[1], m[2]) for m in can_sends if m[0] == 0x2B6]
    if not frames:
      return None
    self.parser.update([[self.now_nanos, frames]])
    return self.parser.vl['HS2_DYN1_MDD_ETAT_2B6']

  def knock_out(self):
    for _ in range(RADAR_DISABLE_FRAME + 1):
      self.CI.update([])
      self.CI.CS.radar_alive = True
      self.CI.apply(structs.CarControl().as_reader(), self.now_nanos)
      self.now_nanos += int(DT_CTRL * 1e9)
    for _ in range(10):
      self.step(long_active=True, accel=0.0)
    assert self.CI.CC.radar_disabled

  def emulated(self, **kwargs):
    """Run frames until the 50 Hz emulation actually emits, and return the decoded message."""
    for _ in range(4):
      msg = self.radar_msg(self.step(**kwargs))
      if msg is not None:
        return msg
    self.fail("emulation never transmitted 0x2B6")

  def emulated_pair(self, **kwargs):
    """Like emulated(), but returns the decoded (0x2B6, 0x2F6) pair from the same frame."""
    for _ in range(4):
      sends = self.step(**kwargs)
      b6 = self.radar_msg(sends)
      if b6 is not None:
        f6 = [(m[0], m[1], m[2]) for m in sends if m[0] == 0x2F6]
        self.parser_2f6.update([[self.now_nanos, f6]])
        return b6, self.parser_2f6.vl['HS2_DYN_MDD_ETAT_2F6']
    self.fail("emulation never transmitted 0x2B6")

  def settled(self, **kwargs):
    """Hold a request until the filters (pitch, torque rate) have caught up with it."""
    for _ in range(int(4.0 / DT_CTRL)):
      self.step(**kwargs)
    return self.emulated(**kwargs)


class TestPsaBrakeOverride(PsaEmulationTest):
  """The emulated radar must never invert its request when the driver takes over.

  The PCM holds cruiseState.enabled up for ~80 ms after a driver brake press, but
  controlsd zeroes actuators.accel as soon as longActive drops. Driving the emulation
  off the PCM alone made those frames advertise an active ACC asking for *positive*
  wheel torque while the car was still decelerating, and the ESP (UC_FREIN) latched
  ACC_ETAT_DECEL_OR_ESP_STATUS = 3 within 30 ms.
  """

  def test_braking_frame_requests_deceleration(self):
    self.knock_out()
    msg = self.settled(long_active=True, accel=-1.25)
    self.assertEqual(int(msg['MDD_DECEL_CONTROL_REQ']), 1)
    self.assertAlmostEqual(msg['MDD_DESIRED_DECELERATION'], -1.25, places=1)
    self.assertEqual(int(msg['POTENTIAL_WHEEL_TORQUE_REQUEST']), 2, "not in brake mode")

  def test_driver_brake_never_inverts_into_a_torque_request(self):
    self.knock_out()
    self.settled(long_active=True, accel=-1.25)

    # driver brakes: longActive drops and accel is zeroed, but the PCM has not caught up yet
    msg = self.emulated(long_active=False, accel=0.0, brake_pressed=True, cruise_enabled=True)
    self.assertNotEqual(int(msg['POTENTIAL_WHEEL_TORQUE_REQUEST']), 1,
                        "asked the ESP for wheel torque while the driver was braking")
    self.assertEqual(msg['GMP_WHEEL_TORQUE'], -4000, "sent a torque value instead of the no-request sentinel")
    self.assertEqual(msg['GMP_POTENTIAL_WHEEL_TORQUE'], -4000)

  def test_release_is_atomic(self):
    # the stock radar releases an active deceleration by dropping every signal to idle in
    # the same frame — a partial release (the off/suspended status with the decel request
    # still up, or the request cleared with a stale desired decel) latched the ESP fault
    self.knock_out()
    self.settled(long_active=True, accel=-1.6)

    msg = self.emulated(long_active=False, accel=0.0, brake_pressed=True, cruise_enabled=True)
    self.assertEqual(int(msg['MDD_DECEL_CONTROL_REQ']), 0)
    self.assertAlmostEqual(msg['MDD_DESIRED_DECELERATION'], 2.05, places=1)
    self.assertEqual(int(msg['ACC_STATUS']), 2, "brake release must advertise ACC off with the request down")

  def test_gas_release_is_atomic(self):
    self.knock_out()
    self.settled(long_active=True, accel=-1.6)

    msg = self.emulated(long_active=True, accel=-1.6, gas_pressed=True)
    self.assertEqual(int(msg['MDD_DECEL_CONTROL_REQ']), 0)
    self.assertAlmostEqual(msg['MDD_DESIRED_DECELERATION'], 2.05, places=1)
    self.assertEqual(int(msg['ACC_STATUS']), 5, "gas over a decel suspends ACC, with the request down")

  def test_brake_press_while_still_enabled_keeps_acc_advertised_active(self):
    # brakePressed goes true a frame before controlsd drops longActive, so the off status
    # must wait for the decel request to clear: one frame of ACC_STATUS 2 with the decel
    # request still up latched the ESP fault
    self.knock_out()
    self.settled(long_active=True, accel=-1.25)

    msg = self.emulated(long_active=True, accel=-1.25, brake_pressed=True)
    self.assertEqual(int(msg['MDD_DECEL_CONTROL_REQ']), 1)
    self.assertEqual(int(msg['ACC_STATUS']), 4, "advertised ACC off while still requesting deceleration")

  def test_standstill_hold_and_drive_away(self):
    # the ESP holds the car at a stop for the radar and only releases after the stock
    # drive-away handshake: hold pattern (saturated -10.65 decel code, no wheel torque),
    # DRIVE_AWAY_REQUEST pulsed in 0x2F6, then the launch — desired decel at the saturated
    # positive end with the decel request still active, which is what the ESP lets go on
    self.knock_out()
    self.emulated(long_active=True, accel=-1.0, v_ego=1.0)

    # stopped: hold pattern, advertising the torque the powertrain is holding in reserve
    msg = self.emulated(long_active=True, accel=-0.2, standstill=True)
    self.assertEqual(int(msg['MDD_DECEL_CONTROL_REQ']), 1)
    self.assertAlmostEqual(msg['MDD_DESIRED_DECELERATION'], -10.65, places=1)
    self.assertEqual(int(msg['WHEEL_TORQUE_REQUEST']), 0)
    self.assertEqual(int(msg['ACC_STATUS']), 4)
    self.assertGreater(msg['GMP_POTENTIAL_WHEEL_TORQUE'], -4000, "hold did not advertise a real potential torque")

    # planner wants to move: the pulse goes up while 0x2B6 stays in the hold pattern
    b6, f6 = self.emulated_pair(long_active=True, accel=0.3, standstill=True)
    self.assertEqual(int(f6['DRIVE_AWAY_REQUEST']), 1)
    self.assertAlmostEqual(b6['MDD_DESIRED_DECELERATION'], -10.65, places=1)
    self.assertEqual(int(b6['WHEEL_TORQUE_REQUEST']), 0)

    # pulse over: launch, wheel torque up with the decel request still active
    for _ in range(100):
      b6, f6 = self.emulated_pair(long_active=True, accel=0.3, standstill=True)
    self.assertEqual(int(f6['DRIVE_AWAY_REQUEST']), 0)
    self.assertEqual(int(b6['WHEEL_TORQUE_REQUEST']), 1)
    self.assertGreater(b6['GMP_WHEEL_TORQUE'], 0, "launched with no positive wheel torque")
    self.assertEqual(int(b6['MDD_DECEL_CONTROL_REQ']), 1)
    self.assertAlmostEqual(b6['MDD_DESIRED_DECELERATION'], 2.0, places=1)

    # rolling: back to the plain torque pattern
    b6, f6 = self.emulated_pair(long_active=True, accel=0.3, v_ego=1.0)
    self.assertEqual(int(b6['MDD_DECEL_CONTROL_REQ']), 0)
    self.assertEqual(int(b6['WHEEL_TORQUE_REQUEST']), 1)

  def test_gas_press_ends_the_standstill_hold(self):
    # the driver pulling away by pedal is an override: keeping the hold up against it left
    # a decel request on the bus while the car crept forward and the ESP latched the fault
    self.knock_out()
    self.emulated(long_active=True, accel=-1.0, v_ego=1.0)
    msg = self.emulated(long_active=True, accel=-0.2, standstill=True)
    self.assertAlmostEqual(msg['MDD_DESIRED_DECELERATION'], -10.65, places=1)

    # gas press: longActive drops and accel is zeroed, cruise stays on
    msg = self.emulated(long_active=False, accel=0.0, gas_pressed=True, standstill=True)
    self.assertEqual(int(msg['MDD_DECEL_CONTROL_REQ']), 0, "held the ESP while the driver was on the pedal")
    self.assertNotAlmostEqual(msg['MDD_DESIRED_DECELERATION'], -10.65, places=1)
    self.assertEqual(int(msg['ACC_STATUS']), 5)

  def test_hold_releases_on_a_descent(self):
    # the hold is driven by the planner's request, not the pitch-compensated one: on a
    # descent gravity alone keeps the compensated value negative and the car would sit
    # there however hard the planner asked to move
    self.knock_out()
    self.emulated(long_active=True, accel=-1.0, v_ego=1.0, pitch=-0.1)
    self.emulated(long_active=True, accel=-0.2, standstill=True, pitch=-0.1)

    b6, f6 = self.emulated_pair(long_active=True, accel=0.3, standstill=True, pitch=-0.1)
    self.assertEqual(int(f6['DRIVE_AWAY_REQUEST']), 1, "hold never released on a downhill grade")

  def test_gas_override_suspends_acc_instead_of_turning_it_off(self):
    # a gas press drops longActive but cruise stays on; the stock radar advertises ACC
    # suspended (ACC_STATUS 5), not the off pattern, so the cluster must not flap
    self.knock_out()
    self.emulated(long_active=True, accel=0.5)

    msg = self.emulated(long_active=False, accel=0.0, gas_pressed=True, cruise_enabled=True)
    self.assertEqual(int(msg['ACC_STATUS']), 5, "advertised ACC off during a gas override")

  def test_gas_and_brake_together_never_request_torque(self):
    # both pedals: the brake wins, the gas override must not keep the enabled pattern up
    self.knock_out()
    self.settled(long_active=True, accel=-1.25)

    msg = self.emulated(long_active=False, accel=0.0, gas_pressed=True, brake_pressed=True, cruise_enabled=True)
    self.assertNotEqual(int(msg['POTENTIAL_WHEEL_TORQUE_REQUEST']), 1)
    self.assertEqual(msg['GMP_WHEEL_TORQUE'], -4000)
    self.assertNotEqual(int(msg['ACC_STATUS']), 5, "kept ACC suspended while the driver was braking")


class TestPsaBrakeRequest(PsaEmulationTest):
  """The brake request is the plan, with no rate limit of its own on the way in or out."""

  def test_brake_request_goes_out_in_the_frame_it_is_asked_for(self):
    self.knock_out()
    msg = self.emulated(long_active=True, accel=-1.2, v_ego=30.0, long_state=LongCtrlState.pid)
    self.assertEqual(int(msg['MDD_DECEL_CONTROL_REQ']), 1)
    self.assertAlmostEqual(msg['MDD_DESIRED_DECELERATION'], -1.2, places=1)

  def test_release_goes_out_in_the_frame_it_is_asked_for(self):
    # the ESP needs the hand-back in the frame it is asked for
    self.knock_out()
    self.settled(long_active=True, accel=-1.2)
    msg = self.emulated(long_active=True, accel=0.0)
    self.assertEqual(int(msg['MDD_DECEL_CONTROL_REQ']), 0, "held a deceleration request after it was released")

  def test_decel_is_kept_inside_what_safety_accepts(self):
    self.knock_out()
    msg = self.emulated(long_active=True, accel=-6.0, v_ego=30.0)
    self.assertAlmostEqual(msg['MDD_DESIRED_DECELERATION'], P.ACCEL_MIN, places=1)

  def test_a_positive_spike_is_not_walked_down(self):
    self.knock_out()
    self.step(long_active=True, accel=1.5, v_ego=11.0)
    msg = self.emulated(long_active=True, accel=0.0, v_ego=11.0)
    self.assertLess(msg['GMP_WHEEL_TORQUE'], 200, "the spike's torque was still on the bus")


class TestPsaBrakeHysteresis(PsaEmulationTest):
  def braking(self, accel):
    return int(self.settled(long_active=True, accel=accel, v_ego=15.0)['MDD_DECEL_CONTROL_REQ']) == 1

  ON, OFF = P.BRAKE_ON_ACCEL, P.BRAKE_OFF_ACCEL

  def test_brakes_engage_below_the_on_threshold_and_hold_until_the_off_threshold(self):
    between = (self.ON + self.OFF) / 2
    self.knock_out()
    self.assertFalse(self.braking(between), "braked between the thresholds coming from torque")
    self.assertTrue(self.braking(self.ON - 0.1))
    self.assertTrue(self.braking(between), "left the brakes inside the hysteresis band")
    self.assertFalse(self.braking(self.OFF + 0.05))


class TestPsaEvBrakeHysteresis(TestPsaBrakeHysteresis):
  EV = True
  ON, OFF = P.EV_BRAKE_ON_ACCEL, P.EV_BRAKE_OFF_ACCEL


class TestPsaEv(PsaEmulationTest):
  EV = True

  def test_only_cars_with_0x202_on_the_adas_bus_are_evs(self):
    self.assertTrue(self.CI.CP.flags & PSAFlags.EV)
    self.assertFalse(make_car_interface(alpha_long=True).CP.flags & PSAFlags.EV)

  def test_torque_request_is_rate_limited(self):
    self.knock_out()
    for _ in range(P.EV_LIFT_OFF_FRAMES):
      self.step(long_active=True, accel=0.0, v_ego=15.0)
    torques = [self.emulated(long_active=True, accel=2.0, v_ego=15.0)['GMP_WHEEL_TORQUE'] for _ in range(20)]
    steps = np.diff(torques)
    self.assertGreater(steps.min(), 0, "the request stopped rising short of the target")
    # the emulation goes out every second frame
    self.assertLessEqual(steps.max(), 2 * P.EV_TORQUE_RATE + 1)

  def test_no_regen_request_right_after_engaging(self):
    self.knock_out()
    accel = P.EV_BRAKE_ON_ACCEL + 0.1  # negative torque, still short of the friction brakes
    self.assertGreaterEqual(self.emulated(long_active=True, accel=accel, v_ego=15.0)['GMP_WHEEL_TORQUE'], P.EV_LIFT_OFF_TORQUE_MIN)
    for _ in range(P.EV_LIFT_OFF_FRAMES + 100):
      self.step(long_active=True, accel=accel, v_ego=15.0)
    self.assertLess(self.emulated(long_active=True, accel=accel, v_ego=15.0)['GMP_WHEEL_TORQUE'], 0)


class TestPsaLateralEngage(unittest.TestCase):
  def setUp(self):
    self.CI = make_car_interface(alpha_long=False)
    self.parser = CANParser('psa_aee2010_r3', [('LANE_KEEP_ASSIST', 0)], 0)
    self.now_nanos = 0

  def lka(self, angle, eps_state=0, driver_torque=0.0, lat_active=True):
    CC = structs.CarControl()
    CC.latActive = lat_active
    CC.actuators.steeringAngleDeg = angle
    self.CI.update([])
    self.CI.CS.out.vEgoRaw = 25.0
    self.CI.CS.out.steeringAngleDeg = angle
    self.CI.CS.out.steeringTorque = driver_torque
    self.CI.CS.eps_state = eps_state
    _, can_sends = self.CI.apply(CC.as_reader(), self.now_nanos)
    self.now_nanos += int(DT_CTRL * 1e9)
    self.parser.update([[self.now_nanos, [m for m in can_sends if m[0] == 0x3F2]]])
    return self.parser.vl['LANE_KEEP_ASSIST']

  def lka_active(self, angle):
    return self.lka(angle)['TORQUE_FACTOR'] > 0

  def test_torque_factor_winds_down_after_disengage(self):
    for _ in range(100):
      self.lka(0.0, eps_state=3)
    # the wheel has moved on by the time lateral drops: the command follows it, the assist ramps out
    frames = [self.lka(7.0, eps_state=3, lat_active=False) | {} for _ in range(60)]
    factors = [f['TORQUE_FACTOR'] for f in frames]
    self.assertEqual(factors[:3], [100 - n * P.TORQUE_FACTOR_STEP_DOWN for n in (1, 2, 3)])
    self.assertEqual(factors[-1], 0)
    self.assertTrue(all(abs(f['SET_ANGLE'] - 7.0) < 0.05 for f in frames), "wind-down command left the measured angle")
    self.assertTrue(all(f['STATUS'] == (4 if f['TORQUE_FACTOR'] else 2) for f in frames))

  def test_torque_factor_yields_to_the_driver(self):
    lo, hi = P.TORQUE_FACTOR_DRIVER_BP
    hands_off = [0] + [self.lka(0.0)['TORQUE_FACTOR'] for _ in range(100)]
    pushing = [self.lka(0.0, driver_torque=-hi)['TORQUE_FACTOR'] for _ in range(100)]
    halfway = [self.lka(0.0, driver_torque=(lo + hi) / 2)['TORQUE_FACTOR'] for _ in range(100)]
    self.assertEqual(hands_off[-1], P.TORQUE_FACTOR_MAX)
    self.assertEqual(pushing[-1], P.TORQUE_FACTOR_MIN)
    self.assertEqual(halfway[-1], (P.TORQUE_FACTOR_MAX + P.TORQUE_FACTOR_MIN) // 2)
    self.assertGreater(P.TORQUE_FACTOR_MIN, 0, "safety reads a zero factor as inactive")
    steps = np.diff(hands_off + pushing + halfway)
    self.assertLessEqual(max(steps), P.TORQUE_FACTOR_STEP_UP, "assist rose faster than the ramp")
    self.assertGreaterEqual(min(steps), -P.TORQUE_FACTOR_STEP_DOWN, "assist fell faster than the ramp")

  def test_eps_activation_steps_are_held(self):
    # the EPS stays unauthorized: READY -> AUTHORIZED -> ACTIVE -> READY, each held for STATUS_STEP frames
    status = [int(self.lka(0.0)['STATUS']) for _ in range(6 * P.STATUS_STEP)]
    changes = [i for i in range(1, len(status)) if status[i] != status[i - 1]]
    self.assertEqual(set(np.diff(changes)), {P.STATUS_STEP})
    self.assertEqual(set(status), {2, 3, 4})

  def test_eps_authorized_or_active_holds_active(self):
    for eps_state in (1, 3):
      for _ in range(3 * P.STATUS_STEP):
        self.assertEqual(int(self.lka(0.0, eps_state)['STATUS']), 4)

  def test_no_takeover_of_a_curve_beyond_the_lateral_accel_limit(self):
    # at 25 m/s 3 m/s^2 of lateral accel is ~16 deg at the wheel
    self.assertFalse(self.lka_active(60.0), "took over a wheel steered beyond what the limiter can hold")
    self.assertTrue(self.lka_active(5.0))
    self.assertTrue(self.lka_active(60.0), "dropped out of a curve it was already steering")


class TestPsaCancelAcc(unittest.TestCase):
  """With stock ACC, a cancel request follows each BSI cruise frame with the frame the BSI sends once ACC is off."""

  ACTIVE = {'COUNTER': 0, 'LONGITUDINAL_REGULATION_TYPE': 3, 'SPEED_SETPOINT': 68, 'CHECKSUM_CONS_RVV_LVV2': 3,
            'RVV_ACC_ACTIVATION_REQ': 1, 'INTER_VEHICLE_TIME_SETPOINT': 2.0}

  def cancel_frames(self, alpha_long: bool, cancel: bool, counters):
    CI = make_car_interface(alpha_long=alpha_long)
    parser = CANParser('psa_aee2010_r3', [('HS2_DAT_MDD_CMD_452', 0)], PSA_ADAS_BUS)
    CC = structs.CarControl()
    CC.cruiseControl.cancel = cancel
    sent, now_nanos = [], 0
    for counter in counters:
      CI.update([])
      CI.CS.hs2_dat_mdd_cmd_452 = self.ACTIVE | {'COUNTER': counter}
      _, can_sends = CI.apply(CC.as_reader(), now_nanos)
      now_nanos += int(DT_CTRL * 1e9)
      for msg in (m for m in can_sends if m[0] == 0x452):
        self.assertEqual(msg[2], PSA_ADAS_BUS)
        parser.update([[now_nanos, [msg]]])
        sent.append(dict(parser.vl['HS2_DAT_MDD_CMD_452']))
    return sent

  def test_one_cancel_frame_per_bsi_frame(self):
    # the BSI sends at 20 Hz, so its counter holds for five control frames
    sent = self.cancel_frames(alpha_long=False, cancel=True, counters=[3] * 5 + [4] * 5 + [15] * 5)
    self.assertEqual([int(m['COUNTER']) for m in sent], [4, 5, 0], "one frame after each BSI frame, on the next counter")
    for m in sent:
      self.assertEqual(m['RVV_ACC_ACTIVATION_REQ'], 0)
      self.assertEqual(m['SPEED_SETPOINT'], 255)
      self.assertEqual(m['CHECKSUM_CONS_RVV_LVV2'], 0)
      self.assertAlmostEqual(m['INTER_VEHICLE_TIME_SETPOINT'], 6.2)
      self.assertEqual(m['LONGITUDINAL_REGULATION_TYPE'], 3, "the rest of the BSI frame is passed through")

  def test_nothing_without_a_cancel_request(self):
    self.assertEqual(self.cancel_frames(alpha_long=False, cancel=False, counters=range(16)), [])

  def test_not_with_openpilot_longitudinal(self):
    # there the radar is ours, a frame for the stock radar has nowhere to go
    self.assertEqual(self.cancel_frames(alpha_long=True, cancel=True, counters=range(16)), [])


class TestPsaCancelAccEmulation(PsaEmulationTest):
  """With openpilot longitudinal the cancel goes through the radar status we emulate."""

  def test_inhibited_while_the_bsi_still_requests_acc(self):
    # openpilot is not engaged and the car's cruise is still on: inhibited, as the stock radar reports
    # when it stops serving a request, never off
    self.knock_out()
    msg = self.emulated(long_active=False, accel=0.0, cruise_enabled=True, cancel=True, v_ego=15.0)
    self.assertEqual(msg['ACC_STATUS'], 2)
    self.assertEqual(msg['GMP_WHEEL_TORQUE'], -4000)
    self.assertEqual(msg['WHEEL_TORQUE_REQUEST'], 0)
    self.assertEqual(msg['MDD_DECEL_CONTROL_REQ'], 0)

  def test_off_once_the_bsi_has_withdrawn_its_request(self):
    self.knock_out()
    msg = self.emulated(long_active=False, accel=0.0, cruise_enabled=False, cancel=False, v_ego=15.0)
    self.assertEqual(msg['ACC_STATUS'], 3)

  def test_gas_override_stays_suspended(self):
    # a gas press with openpilot disengaged keeps the suspended status until the pedal is released
    self.knock_out()
    msg = self.emulated(long_active=False, accel=0.0, cruise_enabled=True, cancel=True, gas_pressed=True, v_ego=15.0)
    self.assertEqual(msg['ACC_STATUS'], 5)
    msg = self.emulated(long_active=False, accel=0.0, cruise_enabled=True, cancel=True, gas_pressed=False, v_ego=15.0)
    self.assertEqual(msg['ACC_STATUS'], 2)

  def test_engaged_driving_is_unchanged(self):
    self.knock_out()
    msg = self.emulated(long_active=True, accel=0.5, cruise_enabled=True, cancel=False, v_ego=15.0)
    self.assertEqual(msg['ACC_STATUS'], 4)


class TestPsaButtonEvents(unittest.TestCase):
  """The stalk buttons are not on CAN, they are read off the set speed and time gap the BSI sends."""

  ALPHA_LONG = False

  def setUp(self):
    self.CI = make_car_interface(alpha_long=self.ALPHA_LONG)
    self.packer = CANPacker('psa_aee2010_r3')
    self.now_nanos = 0

  def buttons(self, kph: int, gap: float, frames: int = 5):
    """Hold a set speed and time gap on the ADAS bus for a few frames, return (type, pressed) of every event."""
    events = []
    for _ in range(frames):
      self.now_nanos += int(DT_CTRL * 1e9)
      msg = self.packer.make_can_msg('HS2_DAT_MDD_CMD_452', PSA_ADAS_BUS, {'SPEED_SETPOINT': kph, 'INTER_VEHICLE_TIME_SETPOINT': gap})
      new = [(str(b.type), b.pressed) for b in self.CI.update([(self.now_nanos, [msg])]).buttonEvents]
      if ('gapAdjustCruise', False) in new and self.ALPHA_LONG:
        # what selfdrived does with a distance button release, and controlsd reports back as bars
        self.CI.CS.personality_bars = (self.CI.CS.personality_bars - 2) % 3 + 1
      events += new
    return events

  def test_set_speed_and_gap_changes_are_short_presses(self):
    self.assertEqual(self.buttons(255, 6.2), [], "ACC off")
    self.assertEqual(self.buttons(68, 2.0), [], "switching ACC on is not a button openpilot knows")
    self.assertEqual(self.buttons(69, 2.0), [('accelCruise', True), ('accelCruise', False)])
    self.assertEqual(self.buttons(64, 2.0), [('decelCruise', True), ('decelCruise', False)])
    self.assertEqual(self.buttons(64, 1.0), [('gapAdjustCruise', True), ('gapAdjustCruise', False)])
    self.assertEqual(self.buttons(64, 1.0), [], "nothing changed")
    self.assertEqual(self.buttons(255, 6.2), [], "switching ACC off")


class TestPsaTimeGapSync(TestPsaButtonEvents):
  """With openpilot longitudinal the car's time gap sets openpilot's personality, whatever it was before."""

  ALPHA_LONG = True
  PRESS = [('gapAdjustCruise', True), ('gapAdjustCruise', False)]

  def synced(self, gap: float, bars_before: int):
    self.CI.CS.personality_bars = bars_before
    events = self.buttons(68, gap, frames=200)
    self.assertEqual(self.buttons(68, gap, frames=200), [], "kept pressing after the personality matched")
    return events, self.CI.CS.personality_bars

  def test_time_gap_sets_the_personality(self):
    self.buttons(68, 2.0)  # ACC on
    for gap, bars in ((2.0, 3), (1.5, 2), (1.0, 1)):  # relaxed, standard, aggressive
      for bars_before in (1, 2, 3):
        events, bars_after = self.synced(gap, bars_before)
        self.assertEqual(bars_after, bars, f"{gap} s from {bars_before} bars")
        self.assertEqual(events, self.PRESS * ((bars_before - bars) % 3), "a press steps the personality down by one")

  def test_no_press_before_openpilot_reports_its_personality(self):
    self.CI.CS.personality_bars = 0
    self.assertEqual(self.buttons(68, 1.0, frames=200), [])

  def test_no_press_with_acc_off(self):
    self.CI.CS.personality_bars = 2
    self.assertEqual(self.buttons(255, 6.2, frames=200), [])

  def test_set_speed_and_gap_changes_are_short_presses(self):
    # the set speed buttons are unchanged, the gap is synced instead of one press per change
    self.CI.CS.personality_bars = 3
    self.buttons(68, 2.0)
    self.assertEqual(self.buttons(69, 2.0), [('accelCruise', True), ('accelCruise', False)])
    self.assertEqual(self.buttons(64, 2.0), [('decelCruise', True), ('decelCruise', False)])


class TestPsaClusterSetSpeed(unittest.TestCase):
  """openpilot's set speed has to read the same as the number on the dash.

  The bus carries a setpoint 2 km/h below the displayed one, which is what the car regulates
  to, so the display is corrected — see CLUSTER_SETPOINT_OFFSET. The control target is scaled
  by wheelSpeedFactor so we hold the same physical wheel speed stock ACC would.
  """

  def setUp(self):
    self.CI = make_car_interface(alpha_long=True)
    self.packer = CANPacker('psa_aee2010_r3')
    self.now_nanos = 0

  def setpoint(self, kph: int):
    """Put SPEED_SETPOINT on the ADAS bus and return the resulting CarState."""
    values = {'SPEED_SETPOINT': kph, 'RVV_ACC_ACTIVATION_REQ': 1}
    for _ in range(3):  # the parser wants a few frames before the values come through
      self.now_nanos += int(DT_CTRL * 1e9)
      msg = self.packer.make_can_msg('HS2_DAT_MDD_CMD_452', PSA_ADAS_BUS, values)
      CS = self.CI.update([(self.now_nanos, [msg])])
    return CS

  def test_cluster_set_speed_matches_the_dash(self):
    for kph in (50, 82, 130):
      CS = self.setpoint(kph)
      self.assertAlmostEqual(CS.cruiseState.speed * CV.MS_TO_KPH, kph * self.CI.CP.wheelSpeedFactor, places=3,
                             msg="control target no longer follows the value the car regulates to")
      self.assertAlmostEqual(CS.cruiseState.speedCluster * CV.MS_TO_KPH, kph + CLUSTER_SETPOINT_OFFSET, places=3,
                             msg="the displayed set speed is a step off the dash")

  def test_no_offset_with_acc_off(self):
    # the signal parks at 255 with ACC off, and 257 is not a set speed anyone should be shown
    CS = self.setpoint(255)
    self.assertEqual(CS.cruiseState.speedCluster, CS.cruiseState.speed)


class TestPsaCruiseState(unittest.TestCase):
  def setUp(self):
    self.CI = make_car_interface(alpha_long=True)
    self.packer = CANPacker('psa_aee2010_r3')
    self.now_nanos = 0

  def update(self, frames: int, regulation_type: int = 3, acc_go: int = 1):
    for _ in range(frames):
      self.now_nanos += int(DT_CTRL * 1e9)
      msgs = [self.packer.make_can_msg('HS2_DAT_MDD_CMD_452', PSA_ADAS_BUS, {'RVV_ACC_ACTIVATION_REQ': 1, 'LONGITUDINAL_REGULATION_TYPE': regulation_type}),
              self.packer.make_can_msg('NEW_MSG_318', PSA_ADAS_BUS, {'ACC_GO_RELATED': acc_go})]
      CS = self.CI.update([(self.now_nanos, msgs)])
    return CS

  def test_only_plain_cruise_and_the_limiter_are_the_wrong_mode(self):
    for regulation_type, non_adaptive in ((0, False), (1, True), (2, True), (3, False)):
      self.assertEqual(self.update(3, regulation_type).cruiseState.nonAdaptive, non_adaptive)

  def test_cmm_dropping_our_torque_faults_after_a_second(self):
    self.assertFalse(self.update(ACC_TORQUE_LOST_FRAMES + 50).accFaulted)
    self.assertFalse(self.update(ACC_TORQUE_LOST_FRAMES - 10, acc_go=0).accFaulted, "faulted on a brief drop")
    self.assertTrue(self.update(20, acc_go=0).accFaulted)
    self.assertFalse(self.update(3).accFaulted)


if __name__ == "__main__":
  unittest.main()
