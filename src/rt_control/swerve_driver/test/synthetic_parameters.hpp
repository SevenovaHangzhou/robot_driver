#ifndef SWERVE_DRIVER_TEST__SYNTHETIC_PARAMETERS_HPP_
#define SWERVE_DRIVER_TEST__SYNTHETIC_PARAMETERS_HPP_
#include <vector>
#include "rclcpp/parameter.hpp"

// Synthetic test values, not a physical chassis calibration.
inline std::vector<rclcpp::Parameter> synthetic_parameters()
{
  return {
      {"calibration_verified", true},
      {"steering_joints", std::vector<std::string>{"s0", "s1", "s2", "s3"}},
      {"drive_joints", std::vector<std::string>{"d0", "d1", "d2", "d3"}},
      {"steering_encoders", std::vector<std::string>{"e0", "e1", "e2", "e3"}},
      {"module_x", std::vector<double>{0.5, 0.5, -0.5, -0.5}},
      {"module_y", std::vector<double>{0.4, -0.4, 0.4, -0.4}},
      {"wheel_radius", std::vector<double>(4, 0.1)},
      {"steering_min", std::vector<double>(4, -3.14159265358979323846)},
      {"steering_max", std::vector<double>(4, 3.14159265358979323846)},
      {"steering_limit_margin", std::vector<double>(4, 0.05)},
      {"steering_limit_tolerance", std::vector<double>(4, 0.01)},
      {"max_linear_speed", 2.0}, {"max_angular_speed", 3.0}, {"max_wheel_speed", 2.0},
      {"max_wheel_acceleration", 10.0}, {"velocity_deadband", 0.001},
      {"max_encoder_difference", 0.2}, {"max_update_period", 0.02},
      {"slip_residual_threshold", 0.1},
      {"alignment_threshold", 0.3}, {"flip_hysteresis", 0.1}, {"max_steering_slew", 5.0},
      {"translation_heading_epsilon", 1e-6}, {"steering_angle_deadband", 0.001},
      {"feedback_timeout", 0.02}, {"pose_covariance", std::vector<double>(6, 1.0)},
      {"twist_covariance", std::vector<double>(6, 1.0)},
      {"imu_fallback_covariance_scale", 2.0}, {"missing_module_covariance_scale", 4.0},
      {"slip_covariance_scale", 5.0}};
}
#endif
