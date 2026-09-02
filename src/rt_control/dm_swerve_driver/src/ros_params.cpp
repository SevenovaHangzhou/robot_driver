#include "dm_swerve_driver/ros_params.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <iterator>
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

template<std::size_t Size>
[[nodiscard]] std::array<std::uint16_t, Size> to_id_array(
  const std::vector<std::int64_t> & values, const char * name)
{
  if (values.size() != Size) {
    throw std::invalid_argument{
      std::string{name} + " must contain exactly " + std::to_string(Size) + " values"};
  }
  std::array<std::uint16_t, Size> result{};
  for (std::size_t index{0U}; index < values.size(); ++index) {
    if (values[index] < 0 || values[index] > std::numeric_limits<std::uint16_t>::max()) {
      throw std::invalid_argument{std::string{name} + " contains an invalid identifier"};
    }
    result[index] = static_cast<std::uint16_t>(values[index]);
  }
  return result;
}

template<std::size_t Size>
[[nodiscard]] std::vector<std::int64_t> to_integer_vector(
  const std::array<std::uint16_t, Size> & values)
{
  std::vector<std::int64_t> result;
  result.reserve(values.size());
  std::transform(
    values.begin(), values.end(), std::back_inserter(result),
    [](std::uint16_t value) {return static_cast<std::int64_t>(value);});
  return result;
}

void declare_can_and_control(
  rclcpp_lifecycle::LifecycleNode & node, const DriverParameters & defaults)
{
  declare_if_missing(node, "can.interface", defaults.can.interface_name);
  declare_if_missing(node, "can.feedback_deadline_us", defaults.can.feedback_deadline_us);
  declare_if_missing(node, "can.write_timeout_register", defaults.can.write_timeout_register);
  declare_if_missing(node, "can.timeout_register_ms", defaults.can.timeout_register_ms);
  declare_if_missing(node, "control.rate_hz", defaults.control.rate_hz);
  declare_if_missing(node, "control.realtime_priority",
    static_cast<std::int64_t>(defaults.control.realtime_priority));
  declare_if_missing(node, "control.cmd_vel_timeout_s", defaults.control.cmd_vel_timeout_s);
  declare_if_missing(node, "control.hold_steer_on_timeout", defaults.control.hold_steer_on_timeout);
}

void declare_chassis_and_limits(
  rclcpp_lifecycle::LifecycleNode & node, const DriverParameters & defaults)
{
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
  declare_if_missing(node, "chassis.velocity_deadband_mps", defaults.chassis.velocity_deadband_mps);
  declare_if_missing(node, "chassis.align_threshold_rad", defaults.chassis.align_threshold_rad);
  declare_if_missing(node, "limits_fallback.p_max", defaults.limits_fallback.position_max);
  declare_if_missing(node, "limits_fallback.v_max", defaults.limits_fallback.velocity_max);
  declare_if_missing(node, "limits_fallback.t_max", defaults.limits_fallback.torque_max);
}

void declare_module_parameters(
  rclcpp_lifecycle::LifecycleNode & node, const DriverParameters & defaults)
{
  declare_if_missing(node, "steering.gear_ratio", defaults.steering.gear_ratio);
  declare_if_missing(node, "steering.kp", defaults.steering.kp);
  declare_if_missing(node, "steering.kd", defaults.steering.kd);
  declare_if_missing(node, "steering.kff_omega", defaults.steering.kff_omega);
  declare_if_missing(node, "steering.max_ff_speed_radps", defaults.steering.max_ff_speed_radps);
  declare_if_missing(node, "steering.zero_offset_rad", to_vector(defaults.steering.zero_offset_rad));
  declare_if_missing(node, "steering.invert", to_vector(defaults.steering.inverted));
  declare_if_missing(node, "drive.gear_ratio", defaults.drive.gear_ratio);
  declare_if_missing(node, "drive.kd", defaults.drive.kd);
  declare_if_missing(node, "drive.ks", defaults.drive.ks);
  declare_if_missing(node, "drive.kv", defaults.drive.kv);
  declare_if_missing(node, "drive.ka", defaults.drive.ka);
  declare_if_missing(node, "drive.invert", to_vector(defaults.drive.inverted));
}

void declare_motor_and_degradation(
  rclcpp_lifecycle::LifecycleNode & node, const DriverParameters & defaults)
{
  declare_if_missing(
    node, "motors.steering_esc_id", to_integer_vector(defaults.motors.steering_esc_id));
  declare_if_missing(
    node, "motors.steering_mst_id", to_integer_vector(defaults.motors.steering_mst_id));
  declare_if_missing(node, "motors.drive_esc_id", to_integer_vector(defaults.motors.drive_esc_id));
  declare_if_missing(node, "motors.drive_mst_id", to_integer_vector(defaults.motors.drive_mst_id));
  declare_if_missing(node, "safety.feedback_silent_cycles",
    static_cast<std::int64_t>(defaults.safety.feedback_silent_cycles));
  declare_if_missing(node, "safety.reenable_period_s", defaults.safety.reenable_period_s);
  declare_if_missing(node, "odometry.imu_topic", defaults.odometry.imu_topic);
  declare_if_missing(node, "odometry.imu_timeout_s", defaults.odometry.imu_timeout_s);
  declare_if_missing(node, "odometry.publish_tf", defaults.odometry.publish_tf);
  declare_if_missing(node, "odometry.odom_frame", defaults.odometry.odom_frame);
  declare_if_missing(node, "odometry.base_frame", defaults.odometry.base_frame);
  declare_if_missing(node, "odometry.publish_rate_hz", defaults.odometry.publish_rate_hz);
}

void load_can_and_control(
  const rclcpp_lifecycle::LifecycleNode & node, DriverParameters & parameters)
{
  parameters.can.interface_name = node.get_parameter("can.interface").as_string();
  parameters.can.feedback_deadline_us = node.get_parameter("can.feedback_deadline_us").as_int();
  parameters.can.write_timeout_register = node.get_parameter("can.write_timeout_register").as_bool();
  parameters.can.timeout_register_ms = node.get_parameter("can.timeout_register_ms").as_int();
  parameters.control.rate_hz = node.get_parameter("control.rate_hz").as_double();
  parameters.control.realtime_priority =
    static_cast<int>(node.get_parameter("control.realtime_priority").as_int());
  parameters.control.cmd_vel_timeout_s =
    node.get_parameter("control.cmd_vel_timeout_s").as_double();
  parameters.control.hold_steer_on_timeout =
    node.get_parameter("control.hold_steer_on_timeout").as_bool();
}

void load_chassis_and_limits(
  const rclcpp_lifecycle::LifecycleNode & node, DriverParameters & parameters)
{
  auto & chassis = parameters.chassis;
  chassis.wheelbase_m = node.get_parameter("chassis.wheelbase_m").as_double();
  chassis.track_m = node.get_parameter("chassis.track_m").as_double();
  chassis.wheel_radius_m = node.get_parameter("chassis.wheel_radius_m").as_double();
  chassis.max_wheel_speed_mps = node.get_parameter("chassis.max_wheel_speed_mps").as_double();
  chassis.max_linear_speed_mps = node.get_parameter("chassis.max_linear_speed_mps").as_double();
  chassis.max_angular_speed_radps =
    node.get_parameter("chassis.max_angular_speed_radps").as_double();
  chassis.max_wheel_acceleration_mps2 =
    node.get_parameter("chassis.max_wheel_acceleration_mps2").as_double();
  chassis.velocity_deadband_mps = node.get_parameter("chassis.velocity_deadband_mps").as_double();
  chassis.align_threshold_rad = node.get_parameter("chassis.align_threshold_rad").as_double();
  parameters.limits_fallback = MotorLimits{
    node.get_parameter("limits_fallback.p_max").as_double(),
    node.get_parameter("limits_fallback.v_max").as_double(),
    node.get_parameter("limits_fallback.t_max").as_double()};
}

void load_module_parameters(
  const rclcpp_lifecycle::LifecycleNode & node, DriverParameters & parameters)
{
  auto & steering = parameters.steering;
  steering.gear_ratio = node.get_parameter("steering.gear_ratio").as_double();
  steering.kp = node.get_parameter("steering.kp").as_double();
  steering.kd = node.get_parameter("steering.kd").as_double();
  steering.kff_omega = node.get_parameter("steering.kff_omega").as_double();
  steering.max_ff_speed_radps = node.get_parameter("steering.max_ff_speed_radps").as_double();
  steering.zero_offset_rad = to_array<double, kSwerveModuleCount>(
    node.get_parameter("steering.zero_offset_rad").as_double_array(), "steering.zero_offset_rad");
  steering.inverted = to_array<bool, kSwerveModuleCount>(
    node.get_parameter("steering.invert").as_bool_array(), "steering.invert");
  auto & drive = parameters.drive;
  drive.gear_ratio = node.get_parameter("drive.gear_ratio").as_double();
  drive.kd = node.get_parameter("drive.kd").as_double();
  drive.ks = node.get_parameter("drive.ks").as_double();
  drive.kv = node.get_parameter("drive.kv").as_double();
  drive.ka = node.get_parameter("drive.ka").as_double();
  drive.inverted = to_array<bool, kSwerveModuleCount>(
    node.get_parameter("drive.invert").as_bool_array(), "drive.invert");
}

void load_motor_and_degradation(
  const rclcpp_lifecycle::LifecycleNode & node, DriverParameters & parameters)
{
  auto & motors = parameters.motors;
  motors.steering_esc_id = to_id_array<kSwerveModuleCount>(
    node.get_parameter("motors.steering_esc_id").as_integer_array(), "motors.steering_esc_id");
  motors.steering_mst_id = to_id_array<kSwerveModuleCount>(
    node.get_parameter("motors.steering_mst_id").as_integer_array(), "motors.steering_mst_id");
  motors.drive_esc_id = to_id_array<kSwerveModuleCount>(
    node.get_parameter("motors.drive_esc_id").as_integer_array(), "motors.drive_esc_id");
  motors.drive_mst_id = to_id_array<kSwerveModuleCount>(
    node.get_parameter("motors.drive_mst_id").as_integer_array(), "motors.drive_mst_id");
  const auto silent_cycles = node.get_parameter("safety.feedback_silent_cycles").as_int();
  if (silent_cycles < 0) {
    throw std::invalid_argument{"safety.feedback_silent_cycles cannot be negative"};
  }
  parameters.safety.feedback_silent_cycles = static_cast<std::uint64_t>(silent_cycles);
  parameters.safety.reenable_period_s = node.get_parameter("safety.reenable_period_s").as_double();
  auto & odometry = parameters.odometry;
  odometry.imu_topic = node.get_parameter("odometry.imu_topic").as_string();
  odometry.imu_timeout_s = node.get_parameter("odometry.imu_timeout_s").as_double();
  odometry.publish_tf = node.get_parameter("odometry.publish_tf").as_bool();
  odometry.odom_frame = node.get_parameter("odometry.odom_frame").as_string();
  odometry.base_frame = node.get_parameter("odometry.base_frame").as_string();
  odometry.publish_rate_hz = node.get_parameter("odometry.publish_rate_hz").as_double();
}

}  // namespace

void declare_driver_parameters(rclcpp_lifecycle::LifecycleNode & node)
{
  const DriverParameters defaults{default_parameters()};
  declare_can_and_control(node, defaults);
  declare_chassis_and_limits(node, defaults);
  declare_module_parameters(node, defaults);
  declare_motor_and_degradation(node, defaults);
}

DriverParameters load_driver_parameters(const rclcpp_lifecycle::LifecycleNode & node)
{
  DriverParameters parameters{};
  load_can_and_control(node, parameters);
  load_chassis_and_limits(node, parameters);
  load_module_parameters(node, parameters);
  load_motor_and_degradation(node, parameters);
  validate_parameters(parameters);
  return parameters;
}

}  // namespace dm_swerve_driver
