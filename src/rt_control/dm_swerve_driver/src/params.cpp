#include "dm_swerve_driver/params.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace dm_swerve_driver {
namespace {

void require_positive(
  std::vector<std::string> & errors, double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    errors.emplace_back(std::string{name} + " must be finite and positive");
  }
}

void require_nonnegative(
  std::vector<std::string> & errors, double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    errors.emplace_back(std::string{name} + " must be finite and nonnegative");
  }
}

template<std::size_t Size>
void require_nonnegative_array(
  std::vector<std::string> & errors,
  const std::array<double, Size> & values,
  const char * name)
{
  if (std::any_of(values.begin(), values.end(), [](double value) {
      return !std::isfinite(value) || value < 0.0;
    }))
  {
    errors.emplace_back(std::string{name} + " values must be finite and nonnegative");
  }
}

void require_scale(
  std::vector<std::string> & errors, double value, const char * name)
{
  if (!std::isfinite(value) || value < 1.0) {
    errors.emplace_back(std::string{name} + " must be finite and at least one");
  }
}

[[nodiscard]] std::string join_errors(const std::vector<std::string> & errors)
{
  std::ostringstream message;
  for (std::size_t index{0U}; index < errors.size(); ++index) {
    if (index != 0U) {
      message << "; ";
    }
    message << errors[index];
  }
  return message.str();
}

}  // namespace

DriverParameters default_parameters()
{
  return {};
}

SteeringAngleLimits steering_angle_limits(
  const DriverParameters & parameters) noexcept
{
  return SteeringAngleLimits{
    parameters.steering.joint_limit_min_rad,
    parameters.steering.joint_limit_max_rad,
    parameters.steering.joint_limit_margin_rad,
    parameters.steering.joint_limit_tolerance_rad};
}

std::vector<std::string> parameter_errors(const DriverParameters & parameters)
{
  std::vector<std::string> errors;
  require_positive(errors, parameters.control.rate_hz, "control.rate_hz");
  require_positive(errors, parameters.control.cmd_vel_timeout_s, "control.cmd_vel_timeout_s");
  if (parameters.control.realtime_priority < 0 || parameters.control.realtime_priority > 99) {
    errors.emplace_back("control.realtime_priority must be in [0, 99]");
  }

  const auto & chassis = parameters.chassis;
  require_positive(errors, chassis.wheelbase_m, "chassis.wheelbase_m");
  require_positive(errors, chassis.track_m, "chassis.track_m");
  require_positive(errors, chassis.wheel_radius_m, "chassis.wheel_radius_m");
  require_positive(errors, chassis.max_wheel_speed_mps, "chassis.max_wheel_speed_mps");
  require_positive(errors, chassis.max_linear_speed_mps, "chassis.max_linear_speed_mps");
  require_positive(errors, chassis.max_angular_speed_radps, "chassis.max_angular_speed_radps");
  require_positive(
    errors, chassis.max_wheel_acceleration_mps2,
    "chassis.max_wheel_acceleration_mps2");
  require_nonnegative(errors, chassis.velocity_deadband_mps, "chassis.velocity_deadband_mps");
  require_nonnegative(errors, chassis.align_threshold_rad, "chassis.align_threshold_rad");
  if (chassis.align_threshold_rad > kPi / 2.0) {
    errors.emplace_back("chassis.align_threshold_rad must not exceed pi/2");
  }

  const auto & steering = parameters.steering;
  require_positive(errors, steering.gear_ratio, "steering.gear_ratio");
  require_nonnegative(errors, steering.flip_hysteresis_rad, "steering.flip_hysteresis_rad");
  if (std::isfinite(steering.flip_hysteresis_rad) &&
    steering.flip_hysteresis_rad >= kPi / 2.0)
  {
    errors.emplace_back("steering.flip_hysteresis_rad must be less than pi/2");
  }
  require_positive(errors, steering.max_slew_radps, "steering.max_slew_radps");
  if (!valid_steering_angle_limits(steering_angle_limits(parameters))) {
    errors.emplace_back("steering angle range must span [pi, 2*pi]");
  }
  if (std::isfinite(steering.joint_limit_min_rad) &&
    std::isfinite(steering.joint_limit_max_rad) &&
    std::isfinite(steering.joint_limit_margin_rad) &&
    (steering.joint_limit_min_rad + steering.joint_limit_margin_rad > 0.0 ||
    steering.joint_limit_max_rad - steering.joint_limit_margin_rad < 0.0))
  {
    errors.emplace_back("steering angle range must contain zero");
  }
  if (std::any_of(
      steering.zero_offset_rad.begin(), steering.zero_offset_rad.end(),
      [](double value) {return !std::isfinite(value);}))
  {
    errors.emplace_back("steering.zero_offset_rad values must be finite");
  }
  require_positive(errors, parameters.drive.gear_ratio, "drive.gear_ratio");

  if (parameters.safety.feedback_silent_cycles == 0U) {
    errors.emplace_back("safety.feedback_silent_cycles must be positive");
  }
  require_positive(errors, parameters.safety.reenable_period_s, "safety.reenable_period_s");
  if (std::isfinite(parameters.safety.reenable_period_s) &&
    parameters.safety.reenable_period_s < 1.0)
  {
    errors.emplace_back("safety.reenable_period_s must be at least 1 second");
  }
  if (parameters.safety.auto_recovery_limit == 0U) {
    errors.emplace_back("safety.auto_recovery_limit must be positive");
  }

  const auto & odometry = parameters.odometry;
  require_positive(errors, odometry.imu_timeout_s, "odometry.imu_timeout_s");
  require_positive(errors, odometry.publish_rate_hz, "odometry.publish_rate_hz");
  require_positive(errors, odometry.max_imu_yaw_step_rad, "odometry.max_imu_yaw_step_rad");
  if (std::isfinite(odometry.max_imu_yaw_step_rad) &&
    odometry.max_imu_yaw_step_rad > kPi)
  {
    errors.emplace_back("odometry.max_imu_yaw_step_rad must not exceed pi");
  }
  require_nonnegative_array(
    errors, odometry.pose_covariance_diagonal, "odometry.pose_covariance_diagonal");
  require_nonnegative_array(
    errors, odometry.twist_covariance_diagonal, "odometry.twist_covariance_diagonal");
  require_scale(
    errors, odometry.imu_fallback_covariance_scale,
    "odometry.imu_fallback_covariance_scale");
  require_scale(
    errors, odometry.missing_module_covariance_scale,
    "odometry.missing_module_covariance_scale");
  require_positive(
    errors, odometry.slip_residual_threshold,
    "odometry.slip_residual_threshold");
  require_scale(errors, odometry.slip_covariance_scale, "odometry.slip_covariance_scale");
  if (odometry.publish_rate_hz > parameters.control.rate_hz) {
    errors.emplace_back("odometry.publish_rate_hz cannot exceed control.rate_hz");
  }
  if (odometry.imu_topic.empty() || odometry.odom_frame.empty() ||
    odometry.base_frame.empty())
  {
    errors.emplace_back("odometry topic and frame names must not be empty");
  }
  return errors;
}

void validate_parameters(const DriverParameters & parameters)
{
  const auto errors = parameter_errors(parameters);
  if (!errors.empty()) {
    throw std::invalid_argument{join_errors(errors)};
  }
}

std::array<Translation2d, kSwerveModuleCount> module_locations(
  const DriverParameters & parameters)
{
  const double half_length{parameters.chassis.wheelbase_m / 2.0};
  const double half_width{parameters.chassis.track_m / 2.0};
  return {
    Translation2d{half_length, half_width},
    Translation2d{half_length, -half_width},
    Translation2d{-half_length, half_width},
    Translation2d{-half_length, -half_width}};
}

}  // namespace dm_swerve_driver
