#pragma once

#include "opendbc/safety/safety_declarations.h"

static bool tesla_legacy_longitudinal = false;
static unsigned int tesla_legacy_chassis_bus = 0U;

static bool tesla_legacy_stock_aeb = false;

// Only rising edges while controls are not allowed are considered for these systems
static bool tesla_legacy_stock_lkas = false;
static bool tesla_legacy_stock_lkas_prev = false;

static void tesla_legacy_rx_hook(const CANPacket_t *msg) {

  if (msg->bus == 0U) {
    // Steering angle from EPAS_sysStatus: (0.1 * val) - 819.2 in deg.
    if (msg->addr == 0x370U) {
      // Store it 1/10 deg to match steering request
      const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
      update_sample(&angle_meas, angle_meas_new);

      const int hands_on_level = msg->data[4] >> 6;  // EPAS_handsOnLevel
      const int eac_status = msg->data[6] >> 5;       // EPAS_eacStatus
      const int eac_error_code = msg->data[2] >> 4;   // EPAS_eacErrorCode

      // Disengage on normal user override, or if high angle rate fault from user overriding extremely quickly
      steering_disengage = (hands_on_level >= 3) || ((eac_status == 0) && (eac_error_code == 9));
    }
  }

  // Chassis bus signals (bus 1 for HW3 Raven, bus 0 for HW2)
  if (msg->bus == tesla_legacy_chassis_bus) {
    // Vehicle speed from ESP_B: (0.01 * val) * KPH_TO_MS
    if (msg->addr == 0x155U) {
      float speed = ((msg->data[6] | (msg->data[5] << 8)) * 0.01) * KPH_TO_MS;
      UPDATE_VEHICLE_SPEED(speed);
    }

    // Brake pressed from BrakeMessage
    if (msg->addr == 0x20aU) {
      brake_pressed = (((msg->data[0] & 0x0CU) >> 2) != 1U);
    }

    // Cruise state from DI_state
    if (msg->addr == 0x368U) {
      int cruise_state = (msg->data[1] >> 4) & 0x07U;
      bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                            (cruise_state == 3) ||  // STANDSTILL
                            (cruise_state == 4) ||  // OVERRIDE
                            (cruise_state == 6) ||  // PRE_FAULT
                            (cruise_state == 7);    // PRE_CANCEL
      vehicle_moving = cruise_state != 3;  // STANDSTILL
      pcm_cruise_check(cruise_engaged);
    }
  }

  // Autopilot party bus (bus 2) — stock system detection
  if (msg->bus == 2U) {
    // DAS_control: stock AEB detection
    if (msg->addr == 0x2bfU) {
      tesla_legacy_stock_aeb = (msg->data[2] & 0x03U) == 1U;  // AEB_ACTIVE
    }

    // DAS_steeringControl: stock LKAS detection
    if (msg->addr == 0x488U) {
      int steering_control_type = msg->data[2] >> 6;
      bool stock_lkas_now = steering_control_type == 2;  // LANE_KEEP_ASSIST

      // Only consider rising edges while controls are not allowed
      if (stock_lkas_now && !tesla_legacy_stock_lkas_prev && !controls_allowed && !m_mads_state.system_enabled) {
        tesla_legacy_stock_lkas = true;
      }
      if (!stock_lkas_now) {
        tesla_legacy_stock_lkas = false;
      }
      tesla_legacy_stock_lkas_prev = stock_lkas_now;
    }
  }
}


static bool tesla_legacy_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits TESLA_LEGACY_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // NOTE: based off Tesla Model S to match openpilot
  const AngleSteeringParams TESLA_LEGACY_STEERING_PARAMS = {
    .slip_factor = -0.0005666493436310427,  // calc_slip_factor(VM)
    .steer_ratio = 15.,
    .wheelbase = 2.96,
  };

  const LongitudinalLimits TESLA_LEGACY_LONG_LIMITS = {
    .max_accel = 425,       // 2 m/s^2
    .min_accel = 288,       // -3.48 m/s^2
    .inactive_accel = 375,  // 0. m/s^2
  };

  bool tx = true;
  bool violation = false;

  // Steering control: (0.1 * val) - 1638.35 in deg.
  if (msg->addr == 0x488U) {
    // We use 1/10 deg as a unit here
    int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = msg->data[2] >> 6;
    bool steer_control_enabled = steer_control_type == 1;  // ANGLE_CONTROL

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, TESLA_LEGACY_STEERING_LIMITS, TESLA_LEGACY_STEERING_PARAMS)) {
      violation = true;
    }

    bool valid_steer_control_type = (steer_control_type == 0) ||  // NONE
                                    (steer_control_type == 1);    // ANGLE_CONTROL
    if (!valid_steer_control_type) {
      violation = true;
    }

    if (tesla_legacy_stock_lkas) {
      // Don't allow any steering commands when stock LKAS is active
      violation = true;
    }
  }

  // DAS_control: longitudinal control message
  if (msg->addr == 0x2bfU) {
    // No AEB events may be sent by openpilot
    int aeb_event = msg->data[2] & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }

    // Don't send long/cancel messages when the stock AEB system is active
    if (tesla_legacy_stock_aeb) {
      violation = true;
    }

    int raw_accel_max = ((msg->data[6] & 0x1FU) << 4) | (msg->data[5] >> 4);
    int raw_accel_min = ((msg->data[5] & 0x0FU) << 5) | (msg->data[4] >> 3);
    int acc_state = msg->data[1] >> 4;

    if (tesla_legacy_longitudinal) {
      // Prevent both acceleration from being negative, as this could cause the car to reverse after coming to standstill
      if ((raw_accel_max < TESLA_LEGACY_LONG_LIMITS.inactive_accel) && (raw_accel_min < TESLA_LEGACY_LONG_LIMITS.inactive_accel)) {
        violation = true;
      }

      // Don't allow any acceleration limits above the safety limits
      violation |= longitudinal_accel_checks(raw_accel_max, TESLA_LEGACY_LONG_LIMITS);
      violation |= longitudinal_accel_checks(raw_accel_min, TESLA_LEGACY_LONG_LIMITS);
    } else {
      // Can only send cancel longitudinal messages when not controlling longitudinal
      if (acc_state != 13) {  // ACC_CANCEL_GENERIC_SILENT
        violation = true;
      }

      // No actuation is allowed when not controlling longitudinal
      if ((raw_accel_max != TESLA_LEGACY_LONG_LIMITS.inactive_accel) || (raw_accel_min != TESLA_LEGACY_LONG_LIMITS.inactive_accel)) {
        violation = true;
      }
    }
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static bool tesla_legacy_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == 2) {
    // APS_eacMonitor
    if (addr == 0x27dU) {
      block_msg = true;
    }

    // DAS_steeringControl
    if ((addr == 0x488U) && !tesla_legacy_stock_lkas) {
      block_msg = true;
    }

    // DAS_control
    if (tesla_legacy_longitudinal && (addr == 0x2bfU) && !tesla_legacy_stock_aeb) {
      block_msg = true;
    }
  }

  return block_msg;
}

static safety_config tesla_legacy_init(uint16_t param) {
  const int TESLA_LEGACY_FLAG_HW3 = 32;

  // Reset state
  tesla_legacy_stock_aeb = false;
  tesla_legacy_stock_lkas = false;
  tesla_legacy_stock_lkas_prev = false;
  tesla_legacy_chassis_bus = 0U;
  tesla_legacy_longitudinal = false;

#ifdef ALLOW_DEBUG
  const int TESLA_LEGACY_FLAG_LONG_CONTROL = 1;
  tesla_legacy_longitudinal = GET_FLAG(param, TESLA_LEGACY_FLAG_LONG_CONTROL);
#endif

  bool hw3 = GET_FLAG(param, TESLA_LEGACY_FLAG_HW3);
  if (hw3) {
    tesla_legacy_chassis_bus = 1U;
  }

  static const CanMsg TESLA_LEGACY_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},   // DAS_steeringControl
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},   // APS_eacMonitor
  };

  static const CanMsg TESLA_LEGACY_LONG_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},   // DAS_steeringControl
    {0x2bf, 0, 8, .check_relay = true, .disable_static_blocking = true},   // DAS_control
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},   // APS_eacMonitor
  };

  // HW3 Raven: chassis bus = 1
  // NOTE: DAS_control (0x2bf) is on powertrain bus (bus 4), not visible to C3's 3-bus panda.
  // Stock AEB detection is disabled for legacy cars without an external panda.
  static RxCheck tesla_legacy_hw3_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // EPAS_sysStatus
    {.msg = {{0x155, 1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // ESP_B (speed)
    {.msg = {{0x20a, 1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // BrakeMessage
    {.msg = {{0x368, 1, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // DI_state (cruise)
    {.msg = {{0x488, 2, 4, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // DAS_steeringControl (stock LKAS)
  };

  // Default HW2: chassis bus = 0
  // NOTE: DAS_control (0x2bf) is on powertrain bus, not visible without external panda.
  static RxCheck tesla_legacy_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // EPAS_sysStatus
    {.msg = {{0x155, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // ESP_B (speed)
    {.msg = {{0x20a, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // DI_state (cruise)
    {.msg = {{0x488, 2, 4, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // DAS_steeringControl (stock LKAS)
  };

  safety_config ret;
  if (hw3) {
    ret = tesla_legacy_longitudinal ? BUILD_SAFETY_CFG(tesla_legacy_hw3_rx_checks, TESLA_LEGACY_LONG_TX_MSGS)
                                    : BUILD_SAFETY_CFG(tesla_legacy_hw3_rx_checks, TESLA_LEGACY_TX_MSGS);
  } else {
    ret = tesla_legacy_longitudinal ? BUILD_SAFETY_CFG(tesla_legacy_rx_checks, TESLA_LEGACY_LONG_TX_MSGS)
                                    : BUILD_SAFETY_CFG(tesla_legacy_rx_checks, TESLA_LEGACY_TX_MSGS);
  }
  return ret;
}

const safety_hooks tesla_legacy_hooks = {
  .init = tesla_legacy_init,
  .rx = tesla_legacy_rx_hook,
  .tx = tesla_legacy_tx_hook,
  .fwd = tesla_legacy_fwd_hook,
};
