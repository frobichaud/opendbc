#!/usr/bin/env python3
"""Tests for Cooperative Steering on Tesla Legacy (Model S/X HW1/2/3).

Coop steering replaces the panda-level hands_on_level >= 3 hard disengage:
driver torque hands control to the driver (EPAS released), and openpilot
resumes with a delay, an angle-convergence guard, and a blend ramp.
These tests pin down that state machine and its carcontroller integration
(EPAS release during override, resume blend, and the +/-20 deg physical clamp)
so future rebases can't silently regress them.
"""
import unittest
from types import SimpleNamespace

from opendbc.car.tesla.coopsteering import CoopSteering

FPS = CoopSteering.FRAME_RATE
RESUME_FRAMES = int(CoopSteering.RESUME_DELAY * FPS)
BLEND_FRAMES = int(CoopSteering.RESUME_BLEND * FPS)
STRONG_TORQUE = CoopSteering.TORQUE_THRESHOLD + 0.5
EPAS_FEEDBACK_TORQUE = 0.5  # typical EPAS actuation feedback, must never trigger


class TestCoopSteeringStateMachine(unittest.TestCase):
  def setUp(self):
    self.cs = CoopSteering()

  def step(self, n=1, torque=0.0, hands_on=0, lat_active=True, angle_diff=0.0):
    result = False
    for _ in range(n):
      result = self.cs.update(torque, hands_on, lat_active, angle_diff)
    return result

  def test_no_override_when_lateral_inactive(self):
    # even massive driver input must not latch override while lat is inactive
    self.assertFalse(self.step(10, torque=10.0, hands_on=3, lat_active=False))
    self.assertEqual(self.cs.blend_factor, 1.0)

  def test_epas_feedback_torque_ignored(self):
    # EPAS actuation feedback (~0.5 Nm) and light grip must not trigger override
    for torque in (EPAS_FEEDBACK_TORQUE, CoopSteering.TORQUE_THRESHOLD - 0.1):
      with self.subTest(torque=torque):
        self.setUp()
        self.assertFalse(self.step(50, torque=torque))
        self.assertEqual(self.cs.blend_factor, 1.0)

  def test_torque_above_threshold_overrides(self):
    self.assertTrue(self.step(torque=STRONG_TORQUE))
    self.assertEqual(self.cs.blend_factor, 0.0)

  def test_negative_torque_overrides(self):
    self.assertTrue(self.step(torque=-STRONG_TORQUE))

  def test_hands_on_level_overrides_without_torque(self):
    # EPAS's own filtered driver detection triggers regardless of raw torque
    self.assertTrue(self.step(hands_on=3, torque=0.0))

  def test_override_holds_while_input_persists(self):
    for _ in range(10 * FPS):  # 10 seconds of continuous driver input
      self.assertTrue(self.step(torque=STRONG_TORQUE))

  def test_resume_after_delay(self):
    self.step(torque=STRONG_TORQUE)
    # override persists during the resume delay
    self.assertTrue(self.step(RESUME_FRAMES - 1))
    # then releases, entering the blend phase
    self.assertFalse(self.step())
    self.assertTrue(0.0 <= self.cs.blend_factor < 1.0)

  def test_resume_blocked_until_angles_converge(self):
    diverged = CoopSteering.MAX_RESUME_ANGLE_DIFF + 1.0
    self.step(torque=STRONG_TORQUE)
    # desired angle far from physical: override extends past the resume delay
    self.assertTrue(self.step(RESUME_FRAMES * 4, angle_diff=diverged))
    # once the planner converges, resume happens promptly
    self.assertFalse(self.step(2, angle_diff=0.0))

  def test_blend_ramps_to_one(self):
    self.step(torque=STRONG_TORQUE)
    self.step(RESUME_FRAMES)  # release
    last = self.cs.blend_factor
    for _ in range(BLEND_FRAMES):
      self.step()
      self.assertGreaterEqual(self.cs.blend_factor, last)
      last = self.cs.blend_factor
    self.assertEqual(self.cs.blend_factor, 1.0)

  def test_new_input_during_blend_reoverrides(self):
    self.step(torque=STRONG_TORQUE)
    self.step(RESUME_FRAMES)  # release, blending
    self.step(BLEND_FRAMES // 2)
    self.assertTrue(0.0 < self.cs.blend_factor < 1.0)
    # driver grabs the wheel again mid-blend
    self.assertTrue(self.step(torque=STRONG_TORQUE))
    self.assertEqual(self.cs.blend_factor, 0.0)

  def test_lateral_drop_resets_state(self):
    self.step(torque=STRONG_TORQUE)
    self.assertFalse(self.step(lat_active=False))
    # re-activation with no driver input starts clean: no override, no blend
    self.assertFalse(self.step())
    self.assertEqual(self.cs.blend_factor, 1.0)


class TestCoopSteeringCarController(unittest.TestCase):
  """Drives the real Tesla CarController and asserts on the steering commands
  it emits: EPAS release during override, resume blend, and the +/-20 deg
  physical clamp."""

  MAX_ANGLE_GAP = 20.0  # deg, hard clamp in carcontroller.py

  def setUp(self):
    from opendbc.car import Bus
    from opendbc.car.tesla.carcontroller import CarController
    from opendbc.car.tesla.interface import CarInterface

    self.CP = CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3")
    self.CP_SP = CarInterface.get_non_essential_params_sp(self.CP, "TESLA_MODEL_S_HW3")
    dbc_names = {Bus.party: "tesla_can", Bus.pt: "tesla_powertrain"}
    self.cc = CarController(dbc_names, self.CP, self.CP_SP)

    # capture steering commands instead of packing CAN bytes
    self.sent = []
    self.cc.tesla_can.create_steering_control = \
      lambda cntr, angle, active: self.sent.append((angle, active)) or b""
    self.cc.tesla_can.create_steering_allowed = lambda cntr: b""
    self.cc.tesla_can.create_longitudinal_command = lambda *a, **kw: b""

  def run_frames(self, n, desired_angle, physical_angle, torque=0.0, hands_on=0,
                 lat_active=True, v_ego=10.0):
    CC = SimpleNamespace(
      actuators=SimpleNamespace(steeringAngleDeg=desired_angle, accel=0.0,
                                as_builder=lambda: SimpleNamespace(steeringAngleDeg=0.0)),
      latActive=lat_active,
      longActive=False,
      cruiseControl=SimpleNamespace(cancel=False),
    )
    CS = SimpleNamespace(
      out=SimpleNamespace(steeringTorque=torque, steeringAngleDeg=physical_angle,
                          vEgoRaw=v_ego, vEgo=v_ego, steerFaultTemporary=False),
      hands_on_level=hands_on,
      das_control={"DAS_controlCounter": 0},
      cruise_override=False,
    )
    for _ in range(n):
      self.cc.update(CC, SimpleNamespace(), CS, 0)
    return self.sent[-1] if self.sent else None

  def test_override_releases_epas_and_tracks_physical(self):
    # steady driving first
    self.run_frames(20, desired_angle=5.0, physical_angle=5.0)
    # driver applies strong torque: EPAS must be released (active=False)
    # and the command must track the physical wheel position
    angle, active = self.run_frames(10, desired_angle=5.0, physical_angle=30.0,
                                    torque=STRONG_TORQUE)
    self.assertFalse(active)
    self.assertAlmostEqual(angle, 30.0, delta=1.0)

  def test_resume_blends_toward_desired(self):
    # NOTE: update() runs at 100Hz but steering frames are 50Hz (frame % 2),
    # so every steering-frame count is doubled in update() calls
    self.run_frames(20, desired_angle=0.0, physical_angle=10.0, torque=STRONG_TORQUE)
    # driver releases; wait out the resume delay plus a couple steering frames
    self.run_frames(2 * (RESUME_FRAMES + 2), desired_angle=0.0, physical_angle=10.0)
    early_angle, early_active = self.sent[-1]
    self.assertTrue(early_active)
    # early in the blend the command stays near the physical angle
    self.assertGreater(early_angle, 5.0)
    # after the full blend the command converges on the planner's desired angle
    angle, active = self.run_frames(2 * (BLEND_FRAMES + 10), desired_angle=0.0, physical_angle=10.0)
    self.assertTrue(active)
    self.assertLess(abs(angle), 2.0)

  def test_physical_clamp_bounds_command(self):
    # planner wants 90 deg while the wheel is at 0: command must stay within
    # +/-20 deg of the physical position (EPAS fault / violent jerk guard)
    for _ in range(100):
      angle, active = self.run_frames(1, desired_angle=90.0, physical_angle=0.0)
      self.assertLessEqual(abs(angle), self.MAX_ANGLE_GAP + 0.01)

  def test_no_steer_when_lateral_inactive(self):
    angle, active = self.run_frames(10, desired_angle=20.0, physical_angle=0.0,
                                    lat_active=False)
    self.assertFalse(active)


if __name__ == "__main__":
  unittest.main()
