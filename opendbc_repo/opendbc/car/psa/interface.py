from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.values import PSAFlags, PSASafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def release_ecus(self) -> bool:
    # hand the ADAS bus back to the radar ECU openpilot longitudinal knocks out
    return self.CC.release_radar()

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    # only the electric cars send 0x202 on the ADAS bus
    if 0x202 in fingerprint[1]:
      ret.flags |= PSAFlags.EV.value

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]
    if ret.flags & PSAFlags.EPS_CAM_BUS:
      ret.safetyConfigs[0].safetyParam |= PSASafetyFlags.EPS_CAM_BUS.value

    ret.dashcamOnly = False

    # liveDelay only learns above 80 km/h, so this prior dominates. Measured ~0.30 s desired curvature to yaw
    ret.steerActuatorDelay = 0.10
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    # wheel speeds read ~1% below GPS speed
    ret.wheelSpeedFactor = 1.01

    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long
    if alpha_long:
      ret.safetyConfigs[0].safetyParam |= PSASafetyFlags.LONG_CONTROL.value

    # 0.3 s on torque, 0.4-0.7 s on the friction brakes
    ret.longitudinalActuatorDelay = 0.4

    # the integrator picks up drag and the launch, less of it on an EV
    ret.longitudinalTuning.kiBP = [0., 4., 13., 20.]
    ret.longitudinalTuning.kiV = [0.18, 0.15, 0.1, 0.1] if ret.flags & PSAFlags.EV else [0.3, 0.4, 0.5, 0.8]

    return ret
