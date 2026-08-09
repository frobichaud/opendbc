#!/usr/bin/env python3
"""Tests for the Tesla gas-override jerk ramp and set-speed-during-override
rule (longcontrol.py), shared by TeslaCAN (modern) and TeslaCANRaven (legacy).

Pins the fix for the post-gas-override lurch: DAS_jerkMax resets to 0 while
the driver overrides with gas and ramps back at JERK_RATE_UP (1 m/s^3/s),
and DAS_setSpeed stays at V_CRUISE_MAX during the override instead of 0.
"""
import unittest

from opendbc.car.tesla.longcontrol import JerkRamp, LONG_DT, long_set_speed
from opendbc.car.tesla.values import CarControllerParams

UPDATES_PER_SECOND = round(1.0 / LONG_DT)  # DAS_control rate (25 Hz)


class TestJerkRamp(unittest.TestCase):
  def setUp(self):
    self.ramp = JerkRamp()

  def test_zero_during_override(self):
    self.ramp.update(cruise_override=False)
    for _ in range(10):
      self.assertEqual(self.ramp.update(cruise_override=True), 0.0)

  def test_ramps_at_one_per_second(self):
    for _ in range(5):
      self.ramp.update(cruise_override=True)
    jerk = 0.0
    for _ in range(UPDATES_PER_SECOND):
      jerk = self.ramp.update(cruise_override=False)
    self.assertAlmostEqual(jerk, CarControllerParams.JERK_RATE_UP, delta=0.05)

  def test_saturates_at_limit(self):
    for _ in range(10 * UPDATES_PER_SECOND):
      jerk = self.ramp.update(cruise_override=False)
    self.assertEqual(jerk, CarControllerParams.JERK_LIMIT_MAX)

  def test_override_resets_saturated_ramp(self):
    for _ in range(10 * UPDATES_PER_SECOND):
      self.ramp.update(cruise_override=False)
    self.assertEqual(self.ramp.update(cruise_override=True), 0.0)
    # soft start again after the override ends
    self.assertLess(self.ramp.update(cruise_override=False), 0.1)


class TestLongSetSpeed(unittest.TestCase):
  def test_inactive_tracks_v_ego(self):
    self.assertAlmostEqual(long_set_speed(10.0, active=False, accel=0.0, cruise_override=False),
                           36.0, delta=0.01)  # 10 m/s -> 36 kph

  def test_inactive_never_negative(self):
    self.assertEqual(long_set_speed(-1.0, active=False, accel=0.0, cruise_override=False), 0)

  def test_active_decel_without_override_targets_zero(self):
    self.assertEqual(long_set_speed(10.0, active=True, accel=-1.0, cruise_override=False), 0)

  def test_active_decel_during_override_keeps_max(self):
    # the core of the fix: no "target zero" compounding while the driver holds gas
    from opendbc.car.interfaces import V_CRUISE_MAX
    self.assertEqual(long_set_speed(10.0, active=True, accel=-1.0, cruise_override=True), V_CRUISE_MAX)

  def test_active_accel_keeps_max(self):
    from opendbc.car.interfaces import V_CRUISE_MAX
    self.assertEqual(long_set_speed(10.0, active=True, accel=1.0, cruise_override=False), V_CRUISE_MAX)


class TestCanBuildersUseRamp(unittest.TestCase):
  """The two CAN builders must both wire the ramp into DAS_jerkMax."""

  def test_legacy_raven(self):
    from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
    from opendbc.car.tesla.values import CANBUS

    captured = []

    class FakePacker:
      def make_can_msg(self, name, bus, values):
        if name == "DAS_control" and "DAS_controlChecksum" in values:
          captured.append(dict(values))
        return (0, b"\x00" * 8, bus)

    tc = TeslaCANRaven({CANBUS.party: FakePacker(), CANBUS.powertrain: FakePacker()})
    tc.create_longitudinal_command(4, -1.0, 0, 20.0, True, cruise_override=True)
    self.assertEqual(captured[-1]["DAS_jerkMax"], 0.0)
    tc.create_longitudinal_command(4, -1.0, 1, 20.0, True, cruise_override=False)
    self.assertGreater(captured[-1]["DAS_jerkMax"], 0.0)

  def test_modern(self):
    from opendbc.car.tesla.teslacan import TeslaCAN

    captured = []

    class FakePacker:
      def make_can_msg(self, name, bus, values):
        if name == "DAS_control":
          captured.append(dict(values))
        return (0, b"\x00" * 8, bus)

    tc = TeslaCAN(None, FakePacker())
    tc.create_longitudinal_command(4, -1.0, 0, 20.0, True, cruise_override=True)
    self.assertEqual(captured[-1]["DAS_jerkMax"], 0.0)
    tc.create_longitudinal_command(4, -1.0, 1, 20.0, True, cruise_override=False)
    self.assertGreater(captured[-1]["DAS_jerkMax"], 0.0)


if __name__ == "__main__":
  unittest.main()
