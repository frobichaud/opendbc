"""Shared longitudinal command logic for Tesla (modern TeslaCAN and legacy
TeslaCANRaven), extracted so it can be unit tested.

Covers the gas-override jerk ramp and the set-speed-during-override rule
(ported from dzid26/opendbc vtb branch): when the driver releases the gas
after overriding cruise, the PCM transitions OVERRIDE -> ENABLED and a full
JERK_LIMIT_MAX decel caused a jarring lurch. DAS_jerkMax instead resets to 0
during the override and ramps back up at JERK_RATE_UP for a soft start.
"""
from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CarControllerParams

LONG_DT = DT_CTRL * 4  # DAS_control is sent every 4th frame (25 Hz)


def long_set_speed(v_ego: float, active: bool, accel: float, cruise_override: bool) -> float:
  """DAS_setSpeed in kph. During a gas override keep V_CRUISE_MAX instead of 0
  on decel, avoiding the PCM "target zero" compounding on release."""
  from opendbc.car.interfaces import V_CRUISE_MAX

  if active:
    return 0 if (accel < 0 and not cruise_override) else V_CRUISE_MAX
  return max(v_ego * CV.MS_TO_KPH, 0)


class JerkRamp:
  """DAS_jerkMax: 0 while the driver overrides with gas, then ramps up at
  JERK_RATE_UP (m/s^3 per second), saturating at JERK_LIMIT_MAX."""

  def __init__(self):
    self.jerk = 0.0

  def update(self, cruise_override: bool) -> float:
    if cruise_override:
      self.jerk = 0.0
    else:
      self.jerk = min(self.jerk + CarControllerParams.JERK_RATE_UP * LONG_DT,
                      CarControllerParams.JERK_LIMIT_MAX)
    return self.jerk
