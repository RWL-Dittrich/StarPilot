"""The car's time gap is written to LongitudinalPersonality once per change, only with openpilot longitudinal."""
import unittest
from types import SimpleNamespace as NS

from openpilot.starpilot.car.psa.gap_personality import PARAM, GapPersonality


class Params:
  def __init__(self):
    self.writes = []

  def put(self, key, value):
    self.writes.append((key, value))


def car_params(**overrides):
  return NS(**{'brand': 'psa', 'openpilotLongitudinalControl': True, 'passive': False, 'dashcamOnly': False, **overrides})


class TestGapPersonality(unittest.TestCase):
  def setUp(self):
    self.params = Params()
    self.owner = GapPersonality()

  def run_gaps(self, *gaps, CP=None):
    for gap in gaps:
      self.owner.update(CP or car_params(), NS(gap_personality=gap), self.params)
    return [value for _, value in self.params.writes]

  def test_writes_each_change_once(self):
    self.assertEqual(self.run_gaps(2, 2, 2, 1, 1, 0, 0), [2, 1, 0])
    self.assertTrue(all(key == PARAM for key, _ in self.params.writes))

  def test_acc_on_again_rewrites_the_same_gap(self):
    # a personality picked on screen while ACC was off is overridden by the gap on the next ACC on
    self.assertEqual(self.run_gaps(0, None, None, 0), [0, 0])

  def test_acc_off_writes_nothing(self):
    self.assertEqual(self.run_gaps(None, None), [])

  def test_not_qualified(self):
    for CP in (car_params(brand='toyota'), car_params(openpilotLongitudinalControl=False),
               car_params(passive=True), car_params(dashcamOnly=True)):
      self.assertEqual(self.run_gaps(0, 1, 2, CP=CP), [], CP)

  def test_rejects_values_outside_the_enum(self):
    self.assertEqual(self.run_gaps(3, -1, 1.5, 'x'), [])

  def test_car_state_without_the_attribute(self):
    self.owner.update(car_params(), NS(), self.params)
    self.assertEqual(self.params.writes, [])


if __name__ == '__main__':
  unittest.main()
