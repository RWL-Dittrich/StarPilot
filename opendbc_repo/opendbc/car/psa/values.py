from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, ACCELERATION_DUE_TO_GRAVITY
from opendbc.car.lateral import AngleSteeringLimitsVM
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, uds

Ecu = CarParams.Ecu

# The EPS drops out from ~3.1 m/s^2, below the ~3.6 m/s^2 safety allows
MAX_LATERAL_ACCEL = 3.0  # m/s^2
# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06
# Matches safety MAX_LATERAL_JERK
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
  EPS_CAM_BUS = 1  # the EPS is on the camera side of the harness
  EV = 2


class CarControllerParams:
  STEER_STEP = 1  # Angle command is sent at 100 Hz
  STATUS_STEP = 5  # frames per step of the EPS activation sequence
  STEER_DRIVER_ALLOWANCE = 0.5  # Driver intervention threshold, Nm

  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    390,  # deg, EPS max
    MAX_LATERAL_ACCEL=MAX_LATERAL_ACCEL,
    MAX_LATERAL_JERK=MAX_LATERAL_JERK,
    # about what the EPS turns the wheel at hands off, a faster command only runs ahead of the wheel
    MAX_ANGLE_RATE=1.5,  # deg/10ms frame
  )

  # EPS assist (TORQUE_FACTOR, percent), reduced as the driver pushes on the wheel so an override meets a
  # yielding wheel. The EPS leaves LKA at ~2.5 Nm of driver torque whatever the assist
  TORQUE_FACTOR_MAX = 100
  TORQUE_FACTOR_MIN = 5  # never 0 while steering, safety reads 0 as inactive
  TORQUE_FACTOR_DRIVER_BP = [0.5, 1.5]  # Nm of driver torque, max to min
  TORQUE_FACTOR_STEP_DOWN = 10  # per frame
  TORQUE_FACTOR_STEP_UP = 2  # per frame

  # safety rejects decel requests outside this, see PSA_LONG_LIMITS in psa.h
  ACCEL_MIN = -3.5  # m/s^2
  ACCEL_MAX = 2.0  # m/s^2

  # the friction brakes take over below BRAKE_ON_ACCEL and stay on until the request is above BRAKE_OFF_ACCEL
  BRAKE_ON_ACCEL = -0.25  # m/s^2
  BRAKE_OFF_ACCEL = -0.1  # m/s^2
  # an EV regens on a negative torque request, brake only past that
  EV_BRAKE_ON_ACCEL = -0.45  # m/s^2
  EV_BRAKE_OFF_ACCEL = -0.25  # m/s^2

  TORQUE_MAX = 1500  # N.m, the stock radar asks up to ~1480 on steep hills
  TORQUE_MIN = -400  # N.m
  EV_TORQUE_RATE = 10  # N.m per 100 Hz frame
  # keep the request out of the lift-off regen right after engaging or a gas override
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
