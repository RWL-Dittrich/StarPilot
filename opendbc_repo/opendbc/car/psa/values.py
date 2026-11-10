from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, ACCELERATION_DUE_TO_GRAVITY
from opendbc.car.lateral import AngleSteeringLimitsVM
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, uds

Ecu = CarParams.Ecu

# The EPS drops out from ~3.1 m/s^2, below the ~3.6 m/s^2 safety allows, so stay under it.
MAX_LATERAL_ACCEL = 3.0  # m/s^2
# Add extra tolerance for average banked road since safety doesn't have the roll.
# ~3.4 degrees, 6% superelevation. Higher actual roll lowers lateral acceleration.
AVERAGE_ROAD_ROLL = 0.06
# Lower than the ISO 11270 lateral jerk limit (5.0 m/s^3) with bank tolerance, matches safety MAX_LATERAL_JERK
MAX_LATERAL_JERK = 3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)  # ~3.6 m/s^3


class EpsState(IntEnum):
  UNAUTHORIZED = 0
  AUTHORIZED = 1
  AVAILABLE = 2
  ACTIVE = 3
  DEFECT = 4


class PSASafetyFlags(IntFlag):
  LONG_CONTROL = 1
  EPS_CAM_BUS = 2


class PSAFlags(IntFlag):
  # the EPS is on the camera side of the harness
  EPS_CAM_BUS = 1
  # the motor holds the speed with no torque and needs less of it than an engine
  EV = 2


class CarControllerParams:
  STEER_STEP = 1  # Angle command is sent at 100 Hz
  STATUS_STEP = 5  # frames per step of the EPS activation sequence

  # safety rejects decel requests outside this, see PSA_LONG_LIMITS in psa.h
  ACCEL_MIN = -3.5  # m/s^2
  ACCEL_MAX = 2.0  # m/s^2

  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    390,  # deg, EPS max
    MAX_LATERAL_ACCEL=MAX_LATERAL_ACCEL,
    MAX_LATERAL_JERK=MAX_LATERAL_JERK,
    # limit angle rate for low speed comfort, matches previous breakpoint-based rate limits at a standstill
    MAX_ANGLE_RATE=4,  # deg/10ms frame
  )
  STEER_DRIVER_ALLOWANCE = 0.5  # Driver intervention threshold, Nm

  # Below BRAKE_ON_ACCEL the ESP's friction brakes take over from the torque request, and stay on
  # until the request rises above BRAKE_OFF_ACCEL, so a request hovering at the threshold doesn't
  # flip modes every frame. Engine braking runs out around -0.3 m/s^2.
  BRAKE_ON_ACCEL = -0.25  # m/s^2
  BRAKE_OFF_ACCEL = -0.1  # m/s^2
  # an EV regens to a negative torque request, and on the brakes the motor regens fully, so brake only past the regen
  EV_BRAKE_ON_ACCEL = -0.45  # m/s^2
  EV_BRAKE_OFF_ACCEL = -0.25  # m/s^2

  TORQUE_MAX = 1500  # N.m, the stock radar asks up to ~1480 on steep hills
  TORQUE_MIN = -400  # N.m
  # the motor answers within a frame, steps in the request are felt as jerk
  EV_TORQUE_RATE = 10  # N.m per 100 Hz frame
  # an EV motor is in its lift-off regen right after engaging or a gas override, keep the request out of it
  EV_LIFT_OFF_TORQUE_MIN = 0  # N.m
  EV_LIFT_OFF_FRAMES = 50  # 0.5 s


@dataclass
class PSACarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))


@dataclass
class PSAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'psa_aee2010_r3',
  })


class CAR(Platforms):
  PSA_PEUGEOT_208 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 208 2019-25")],
    CarSpecs(mass=1530, wheelbase=2.54, steerRatio=17.6),
  )
  PSA_PEUGEOT_508 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 508 2019-23")],
    CarSpecs(mass=1720, wheelbase=2.79, steerRatio=17.6),
    flags=PSAFlags.EPS_CAM_BUS,
  )


PSA_DIAG_REQ  = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, 0x01])
PSA_DIAG_RESP = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, 0x01])

PSA_SERIAL_REQ = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER,  0xF1, 0x8C])
PSA_SERIAL_RESP = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF1, 0x8C])

PSA_RX_OFFSET = -0x20

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"\d{18}[0-9A-Z]?",
  requests=[
    Request(
      [PSA_DIAG_REQ, PSA_SERIAL_REQ],
      [PSA_DIAG_RESP, PSA_SERIAL_RESP],
      rx_offset=PSA_RX_OFFSET,
      bus=0,
      obd_multiplexing=False,
    )
  ]
)

DBC = CAR.create_dbc_map()
