#ifndef CTRL_SWERVE_TEST__NAVIGATION_FEEDBACK_SUPPORT_HPP_
#define CTRL_SWERVE_TEST__NAVIGATION_FEEDBACK_SUPPORT_HPP_

#include "ctrl_swerve/control_core.hpp"

namespace ctrl_swerve::test
{
// Deliberately not a deployment calibration or the consumer's default geometry.
inline CoreConfig feedback_config()
{
  CoreConfig c;
  c.locations = {{{0.51, 0.39}, {0.51, -0.39}, {-0.51, 0.39}, {-0.51, -0.39}}};
  c.wheel_radius.fill(0.1);
  c.steering_min.fill(-3.0 * kPi / 4.0);
  c.steering_max.fill(3.0 * kPi / 4.0);
  c.steering_limit_margin.fill(kPi / 12.0);
  c.steering_limit_tolerance.fill(0.002);
  c.max_linear_speed = 1.0;
  c.max_angular_speed = 0.8;
  c.max_wheel_speed = 2.0;
  c.max_wheel_acceleration = 2.0;
  c.velocity_deadband = 0.001;
  c.max_encoder_difference = 0.2;
  c.max_update_period = 0.02;
  c.slip_residual_threshold = 0.1;
  c.max_linear_acceleration = 1.0;
  c.max_angular_acceleration = kPi / 8.0;
  c.max_body_jerk = 4.0;
  c.setpoint.alignment_threshold_rad = kPi / 2.0;
  c.setpoint.flip_hysteresis_rad = kPi / 60.0;
  c.setpoint.maximum_steering_slew_radps = kPi / 2.0;
  c.setpoint.maximum_steering_acceleration_radps2 = kPi;
  c.setpoint.maximum_steering_jerk_radps3 = 1.7453292519943295;
  return c;
}

inline ModuleFeedbackArray stopped_feedback()
{
  ModuleFeedbackArray feedback{};
  for (auto & module : feedback) {
    module.valid = module.enabled = module.encoder_valid = true;
  }
  return feedback;
}
}  // namespace ctrl_swerve::test
#endif
