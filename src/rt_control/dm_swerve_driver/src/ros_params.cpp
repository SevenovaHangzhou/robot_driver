#include "dm_swerve_driver/ros_params.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace dm_swerve_driver {
namespace {

template<typename T>
void declare_if_missing(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  const T & default_value)
{
  if (!node.has_parameter(name)) {
    static_cast<void>(node.declare_parameter<T>(name, default_value));
  }
}

template<typename T, std::size_t Size>
[[nodiscard]] std::vector<T> to_vector(const std::array<T, Size> & values)
{
  return std::vector<T>(values.begin(), values.end());
}

template<typename T, std::size_t Size>
[[nodiscard]] std::array<T, Size> to_array(
  const std::vector<T> & values, const char * name)
{
  if (values.size() != Size) {
    throw std::invalid_argument{
      std::string{name} + " must contain exactly " + std::to_string(Size) + " values"};
  }
  std::array<T, Size> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

void declare_control_and_chassis(
  rclcpp_lifecycle::LifecycleNode & node, const DriverParameters & defaults)
{
  declare_if_missing(node, "control.rate_hz", defaults.control.rate_hz);
  declare_if_missing(
    node, "control.realtime_priority",
    static_cast<std::int64_t>(defaults.control.realtime_priority));
  declare_if_missing(node, "control.cmd_vel_timeout_s", defaults.control.cmd_vel_timeout_s);
  declare_if_missing(
    node, "control.hold_steer_on_timeout", defaults.control.hold_steer_on_timeout);
  declare_if_missing(node, "chassis.wheelbase_m", defaults.chassis.wheelbase_m);
  declare_if_missing(node, "chassis.track_m", defaults.chassis.track_m);
  declare_if_missing(node, "chassis.wheel_radius_m", defaults.chassis.wheel_radius_m);
  declare_if_missing(node, "chassis.max_wheel_speed_mps", defaults.chassis.max_wheel_speed_mps);
  declare_if_missing(node, "chassis.max_linear_speed_mps", defaults.chassis.max_linear_speed_mps);
  declare_if_missing(
    node, "chassis.max_angular_speed_radps", defaults.chassis.max_angular_speed_radps);
  declare_if_missing(
    node, "chassis.max_wheel_acceleration_mps2",
    defaults.chassis.max_wheel_acceleration_mps2);
  declare_if_missing(
    node, "chassis.velocity_deadband_mps", defaults.chassis.velocity_deadband_mps);
  declare_if_missing(node, "chassis.align_threshold_rad", defaults.chassis.align_threshold_rad);
}

void declare_steering_and_drive(
  rclcpp_lifecycle::LifecycleNode & node, const DriverParameters & defaults)
{
  declare_if_missing(node, "steering.gear_ratio", defaults.steering.gear_ratio);
  declare_if_missing(
    node, "steering.flip_hysteresis_rad", defaults.steering.flip_hysteresis_rad);
  declare_if_missing(node, "steering.max_slew_radps", defaults.steering.max_slew_radps);
  declare_if_missing(
    node, "steering.joint_limit_min_rad", defaults.steering.joint_limit_min_rad);
  declare_if_missing(
    node, "steering.joint_limit_max_rad", defaults.steering.joint_limit_max_rad);
  declare_if_missing(
    node, "steering.joint_limit_margin_rad", defaults.steering.joint_limit_margin_rad);
  declare_if_missing(
    node, "steering.joint_limit_tolerance_rad",
    defaults.steering.joint_limit_tolerance_rad);
  declare_if_missing(
    node, "steering.zero_offset_rad", to_vector(defaults.steering.zero_offset_rad));
  declare_if_missing(node, "steering.invert", to_vector(defaults.steering.inverted));
  declare_if_missing(node, "drive.gear_ratio", defaults.drive.gear_ratio);
  declare_if_missing(node, "drive.invert", to_vector(defaults.drive.inverted));
}

void declare_safety_and_odometry(
  rclcpp_lifecycle::LifecycleNode & node, const DriverParameters & defaults)
{
  declare_if_missing(
    node, "safety.feedback_silent_cycles",
    static_cast<std::int64_t>(defaults.safety.feedback_silent_cycles));
  declare_if_missing(node, "safety.reenable_period_s", defaults.safety.reenable_period_s);
  declare_if_missing(
    node, "safety.auto_recovery_limit",
    static_cast<std::int64_t>(defaults.safety.auto_recovery_limit));
  declare_if_missing(node, "odometry.imu_topic", defaults.odometry.imu_topic);
  declare_if_missing(node, "odometry.imu_timeout_s", defaults.odometry.imu_timeout_s);
  declare_if_missing(node, "odometry.publish_tf", defaults.odometry.publish_tf);
  declare_if_missing(node, "odometry.odom_frame", defaults.odometry.odom_frame);
  declare_if_missing(node, "odometry.base_frame", defaults.odometry.base_frame);
  declare_if_missing(node, "odometry.publish_rate_hz", defaults.odometry.publish_rate_hz);
  declare_if_missing(
    node, "odometry.max_imu_yaw_step_rad", defaults.odometry.max_imu_yaw_step_rad);
  declare_if_missing(
    node, "odometry.pose_covariance_diagonal",
    to_vector(defaults.odometry.pose_covariance_diagonal));
  declare_if_missing(
    node, "odometry.twist_covariance_diagonal",
    to_vector(defaults.odometry.twist_covariance_diagonal));
  declare_if_missing(
    node, "odometry.imu_fallback_covariance_scale",
    defaults.odometry.imu_fallback_covariance_scale);
  declare_if_missing(
    node, "odometry.missing_module_covariance_scale",
    defaults.odometry.missing_module_covariance_scale);
  declare_if_missing(
    node, "odometry.slip_residual_threshold", defaults.odometry.slip_residual_threshold);
  declare_if_missing(
    node, "odometry.slip_covariance_scale", defaults.odometry.slip_covariance_scale);
}

}  // namespace

void declare_driver_parameters(rclcpp_lifecycle::LifecycleNode & node)
{
  const DriverParameters defaults{default_parameters()};
  declare_control_and_chassis(node, defaults);
  declare_steering_and_drive(node, defaults);
  declare_safety_and_odometry(node, defaults);
}

DriverParameters load_driver_parameters(const rclcpp_lifecycle::LifecycleNode & node)
{
  DriverParameters result;
  result.control.rate_hz = node.get_parameter("control.rate_hz").as_double();
  const auto priority = node.get_parameter("control.realtime_priority").as_int();
  if (priority < 0 || priority > 99) {
    throw std::invalid_argument{"control.realtime_priority must be in [0, 99]"};
  }
  result.control.realtime_priority = static_cast<int>(priority);
  result.control.cmd_vel_timeout_s =
    node.get_parameter("control.cmd_vel_timeout_s").as_double();
  result.control.hold_steer_on_timeout =
    node.get_parameter("control.hold_steer_on_timeout").as_bool();

  auto & chassis = result.chassis;
  chassis.wheelbase_m = node.get_parameter("chassis.wheelbase_m").as_double();
  chassis.track_m = node.get_parameter("chassis.track_m").as_double();
  chassis.wheel_radius_m = node.get_parameter("chassis.wheel_radius_m").as_double();
  chassis.max_wheel_speed_mps = node.get_parameter("chassis.max_wheel_speed_mps").as_double();
  chassis.max_linear_speed_mps = node.get_parameter("chassis.max_linear_speed_mps").as_double();
  chassis.max_angular_speed_radps =
    node.get_parameter("chassis.max_angular_speed_radps").as_double();
  chassis.max_wheel_acceleration_mps2 =
    node.get_parameter("chassis.max_wheel_acceleration_mps2").as_double();
  chassis.velocity_deadband_mps =
    node.get_parameter("chassis.velocity_deadband_mps").as_double();
  chassis.align_threshold_rad = node.get_parameter("chassis.align_threshold_rad").as_double();

  auto & steering = result.steering;
  steering.gear_ratio = node.get_parameter("steering.gear_ratio").as_double();
  steering.flip_hysteresis_rad =
    node.get_parameter("steering.flip_hysteresis_rad").as_double();
  steering.max_slew_radps = node.get_parameter("steering.max_slew_radps").as_double();
  steering.joint_limit_min_rad =
    node.get_parameter("steering.joint_limit_min_rad").as_double();
  steering.joint_limit_max_rad =
    node.get_parameter("steering.joint_limit_max_rad").as_double();
  steering.joint_limit_margin_rad =
    node.get_parameter("steering.joint_limit_margin_rad").as_double();
  steering.joint_limit_tolerance_rad =
    node.get_parameter("steering.joint_limit_tolerance_rad").as_double();
  steering.zero_offset_rad = to_array<double, kSwerveModuleCount>(
    node.get_parameter("steering.zero_offset_rad").as_double_array(),
    "steering.zero_offset_rad");
  steering.inverted = to_array<bool, kSwerveModuleCount>(
    node.get_parameter("steering.invert").as_bool_array(), "steering.invert");
  result.drive.gear_ratio = node.get_parameter("drive.gear_ratio").as_double();
  result.drive.inverted = to_array<bool, kSwerveModuleCount>(
    node.get_parameter("drive.invert").as_bool_array(), "drive.invert");

  const auto silent_cycles = node.get_parameter("safety.feedback_silent_cycles").as_int();
  const auto recovery_limit = node.get_parameter("safety.auto_recovery_limit").as_int();
  if (silent_cycles < 0 || recovery_limit < 0 ||
    recovery_limit > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max()))
  {
    throw std::invalid_argument{"safety integer parameter is outside its valid range"};
  }
  result.safety.feedback_silent_cycles = static_cast<std::uint64_t>(silent_cycles);
  result.safety.reenable_period_s =
    node.get_parameter("safety.reenable_period_s").as_double();
  result.safety.auto_recovery_limit = static_cast<std::uint32_t>(recovery_limit);

  auto & odometry = result.odometry;
  odometry.imu_topic = node.get_parameter("odometry.imu_topic").as_string();
  odometry.imu_timeout_s = node.get_parameter("odometry.imu_timeout_s").as_double();
  odometry.publish_tf = node.get_parameter("odometry.publish_tf").as_bool();
  odometry.odom_frame = node.get_parameter("odometry.odom_frame").as_string();
  odometry.base_frame = node.get_parameter("odometry.base_frame").as_string();
  odometry.publish_rate_hz = node.get_parameter("odometry.publish_rate_hz").as_double();
  odometry.max_imu_yaw_step_rad =
    node.get_parameter("odometry.max_imu_yaw_step_rad").as_double();
  odometry.pose_covariance_diagonal = to_array<double, 6U>(
    node.get_parameter("odometry.pose_covariance_diagonal").as_double_array(),
    "odometry.pose_covariance_diagonal");
  odometry.twist_covariance_diagonal = to_array<double, 6U>(
    node.get_parameter("odometry.twist_covariance_diagonal").as_double_array(),
    "odometry.twist_covariance_diagonal");
  odometry.imu_fallback_covariance_scale =
    node.get_parameter("odometry.imu_fallback_covariance_scale").as_double();
  odometry.missing_module_covariance_scale =
    node.get_parameter("odometry.missing_module_covariance_scale").as_double();
  odometry.slip_residual_threshold =
    node.get_parameter("odometry.slip_residual_threshold").as_double();
  odometry.slip_covariance_scale =
    node.get_parameter("odometry.slip_covariance_scale").as_double();
  validate_parameters(result);
  return result;
}

}  // namespace dm_swerve_driver
