#ifndef DM_SWERVE_DRIVER__PARAMS_HPP_
#define DM_SWERVE_DRIVER__PARAMS_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "dm_swerve_driver/swerve_kinematics.hpp"

namespace dm_swerve_driver {

struct ControlParameters {
  double rate_hz{100.0};
  int realtime_priority{0};
  double cmd_vel_timeout_s{0.25};
  bool hold_steer_on_timeout{true};
};

struct ChassisParameters {
  double wheelbase_m{0.5};
  double track_m{0.4};
  double wheel_radius_m{0.1};
  double max_wheel_speed_mps{2.0};
  double max_linear_speed_mps{1.5};
  double max_angular_speed_radps{3.0};
  double max_wheel_acceleration_mps2{3.0};
  double velocity_deadband_mps{0.01};
  double align_threshold_rad{0.349};
};

struct SteeringParameters {
  double gear_ratio{1.0};
  std::array<double, kSwerveModuleCount> zero_offset_rad{};
  std::array<bool, kSwerveModuleCount> inverted{};
  double flip_hysteresis_rad{0.1};
  double max_slew_radps{3.0};
  double joint_limit_min_rad{-kPi};
  double joint_limit_max_rad{kPi};
  double joint_limit_margin_rad{0.0};
  double joint_limit_tolerance_rad{0.0};
};

struct DriveParameters {
  double gear_ratio{1.0};
  std::array<bool, kSwerveModuleCount> inverted{};
};

struct SafetyParameters {
  std::uint64_t feedback_silent_cycles{50U};
  double reenable_period_s{1.0};
  std::uint32_t auto_recovery_limit{3U};
};

struct OdometryParameters {
  std::string imu_topic{"/imu/data"};
  double imu_timeout_s{0.2};
  bool publish_tf{true};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_link"};
  double publish_rate_hz{50.0};
  double max_imu_yaw_step_rad{0.5};
  std::array<double, 6U> pose_covariance_diagonal{
    0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.02};
  std::array<double, 6U> twist_covariance_diagonal{
    0.02, 0.02, 1000000.0, 1000000.0, 1000000.0, 0.04};
  double imu_fallback_covariance_scale{10.0};
  double missing_module_covariance_scale{4.0};
  double slip_residual_threshold{0.25};
  double slip_covariance_scale{4.0};
};

struct DriverParameters {
  ControlParameters control{};
  ChassisParameters chassis{};
  SteeringParameters steering{};
  DriveParameters drive{};
  SafetyParameters safety{};
  OdometryParameters odometry{};
};

[[nodiscard]] DriverParameters default_parameters();
[[nodiscard]] std::vector<std::string> parameter_errors(
  const DriverParameters & parameters);
void validate_parameters(const DriverParameters & parameters);
[[nodiscard]] SteeringAngleLimits steering_angle_limits(
  const DriverParameters & parameters) noexcept;
[[nodiscard]] std::array<Translation2d, kSwerveModuleCount> module_locations(
  const DriverParameters & parameters);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__PARAMS_HPP_
