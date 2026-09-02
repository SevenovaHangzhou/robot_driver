#include "dm_swerve_driver/params.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace dm_swerve_driver {
namespace {

[[nodiscard]] bool positive_finite(double value) noexcept
{
  return std::isfinite(value) && value > 0.0;
}

void require_positive(
  std::vector<std::string> & errors, double value, const char * name)
{
  if (!positive_finite(value)) {
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

template<typename T, std::size_t Size>
void validate_ids(
  std::vector<std::string> & errors,
  const std::array<T, Size> & ids,
  const char * name)
{
  if (std::any_of(ids.begin(), ids.end(), [](T id) {return id > 0x7FFU;})) {
    errors.emplace_back(std::string{name} + " values must fit in 11 bits");
  }
}

template<typename T, std::size_t Size>
void append_ids(std::set<T> & destination, const std::array<T, Size> & source)
{
  destination.insert(source.begin(), source.end());
}

void validate_motor_ids(
  std::vector<std::string> & errors, const MotorParameters & motors)
{
  validate_ids(errors, motors.steering_esc_id, "steering ESC_ID");
  validate_ids(errors, motors.drive_esc_id, "drive ESC_ID");
  validate_ids(errors, motors.steering_mst_id, "steering MST_ID");
  validate_ids(errors, motors.drive_mst_id, "drive MST_ID");

  std::set<std::uint16_t> esc_ids;
  append_ids(esc_ids, motors.steering_esc_id);
  append_ids(esc_ids, motors.drive_esc_id);
  if (esc_ids.size() != 2U * kSwerveModuleCount) {
    errors.emplace_back("ESC_ID values must be unique across all motors");
  }
  if (esc_ids.count(kRegisterCanId) != 0U) {
    errors.emplace_back("ESC_ID cannot use the register command identifier 0x7FF");
  }

  std::set<std::uint16_t> mst_ids;
  append_ids(mst_ids, motors.steering_mst_id);
  append_ids(mst_ids, motors.drive_mst_id);
  if (mst_ids.size() != 2U * kSwerveModuleCount) {
    errors.emplace_back("MST_ID values must be unique across all motors");
  }
  if (mst_ids.count(kRegisterCanId) != 0U) {
    errors.emplace_back("MST_ID cannot use the register command identifier 0x7FF");
  }
  const bool overlap = std::any_of(
    esc_ids.begin(), esc_ids.end(), [&](std::uint16_t id) {
      return mst_ids.count(id) != 0U;
    });
  if (overlap) {
    errors.emplace_back("ESC_ID and MST_ID sets must not overlap on the CAN bus");
  }
}

void validate_module_index(std::size_t module_index)
{
  if (module_index >= kSwerveModuleCount) {
    throw std::out_of_range{"swerve module index is out of range"};
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

void validate_can_and_control(
  std::vector<std::string> & errors, const DriverParameters & parameters)
{
  if (parameters.can.interface_name.empty()) {
    errors.emplace_back("can.interface must not be empty");
  }
  if (parameters.can.feedback_deadline_us <= 0) {
    errors.emplace_back("can.feedback_deadline_us must be positive");
  }
  if (parameters.can.timeout_register_ms <= 0) {
    errors.emplace_back("can.timeout_register_ms must be positive");
  } else if (parameters.can.timeout_register_ms >
    static_cast<std::int64_t>(
      std::numeric_limits<std::uint32_t>::max() / kTimeoutCountsPerMillisecond))
  {
    errors.emplace_back("can.timeout_register_ms exceeds the register count range");
  }
  require_positive(errors, parameters.control.rate_hz, "control.rate_hz");
  require_positive(errors, parameters.control.cmd_vel_timeout_s, "control.cmd_vel_timeout_s");
  if (parameters.control.realtime_priority < 0 || parameters.control.realtime_priority > 99) {
    errors.emplace_back("control.realtime_priority must be in [0, 99]");
  }
  if (positive_finite(parameters.control.rate_hz)) {
    const double period_us{1.0e6 / parameters.control.rate_hz};
    if (static_cast<double>(parameters.can.feedback_deadline_us) >= period_us) {
      errors.emplace_back("can.feedback_deadline_us must be shorter than the control period");
    }
  }
}

void validate_chassis(
  std::vector<std::string> & errors, const ChassisParameters & chassis)
{
  require_positive(errors, chassis.wheelbase_m, "chassis.wheelbase_m");
  require_positive(errors, chassis.track_m, "chassis.track_m");
  require_positive(errors, chassis.wheel_radius_m, "chassis.wheel_radius_m");
  require_positive(errors, chassis.max_wheel_speed_mps, "chassis.max_wheel_speed_mps");
  require_positive(errors, chassis.max_linear_speed_mps, "chassis.max_linear_speed_mps");
  require_positive(errors, chassis.max_angular_speed_radps, "chassis.max_angular_speed_radps");
  require_positive(
    errors, chassis.max_wheel_acceleration_mps2, "chassis.max_wheel_acceleration_mps2");
  require_nonnegative(errors, chassis.velocity_deadband_mps, "chassis.velocity_deadband_mps");
  require_nonnegative(errors, chassis.align_threshold_rad, "chassis.align_threshold_rad");
  if (chassis.align_threshold_rad > kPi / 2.0) {
    errors.emplace_back("chassis.align_threshold_rad must not exceed pi/2");
  }
}

void validate_steering(
  std::vector<std::string> & errors, const DriverParameters & parameters)
{
  if (!parameters.limits_fallback.valid()) {
    errors.emplace_back("limits_fallback values must be finite and positive");
  }
  const auto & steering = parameters.steering;
  require_positive(errors, steering.gear_ratio, "steering.gear_ratio");
  require_nonnegative(errors, steering.kp, "steering.kp");
  require_nonnegative(errors, steering.kd, "steering.kd");
  require_nonnegative(errors, steering.kff_omega, "steering.kff_omega");
  require_nonnegative(errors, steering.max_ff_speed_radps, "steering.max_ff_speed_radps");
  if (steering.kp > kMitKpMax || steering.kd > kMitKdMax) {
    errors.emplace_back("steering kp/kd exceed MIT field limits");
  }
  if (std::any_of(
      steering.zero_offset_rad.begin(), steering.zero_offset_rad.end(),
      [](double value) {return !std::isfinite(value);}))
  {
    errors.emplace_back("steering.zero_offset_rad values must be finite");
  }
}

void validate_drive_and_degradation(
  std::vector<std::string> & errors, const DriverParameters & parameters)
{
  const auto & drive = parameters.drive;
  require_positive(errors, drive.gear_ratio, "drive.gear_ratio");
  require_nonnegative(errors, drive.kd, "drive.kd");
  require_nonnegative(errors, drive.ks, "drive.ks");
  require_nonnegative(errors, drive.kv, "drive.kv");
  require_nonnegative(errors, drive.ka, "drive.ka");
  if (drive.kd > kMitKdMax) {
    errors.emplace_back("drive.kd exceeds the MIT field limit");
  }
  if (parameters.safety.feedback_silent_cycles == 0U) {
    errors.emplace_back("safety.feedback_silent_cycles must be positive");
  }
  require_positive(errors, parameters.safety.reenable_period_s, "safety.reenable_period_s");
  require_positive(errors, parameters.odometry.imu_timeout_s, "odometry.imu_timeout_s");
  require_positive(errors, parameters.odometry.publish_rate_hz, "odometry.publish_rate_hz");
  if (parameters.odometry.publish_rate_hz > parameters.control.rate_hz) {
    errors.emplace_back("odometry.publish_rate_hz cannot exceed control.rate_hz");
  }
  if (parameters.odometry.imu_topic.empty() || parameters.odometry.odom_frame.empty() ||
    parameters.odometry.base_frame.empty())
  {
    errors.emplace_back("odometry topic and frame names must not be empty");
  }
}

}  // namespace

DriverParameters default_parameters()
{
  return {};
}

std::vector<std::string> parameter_errors(const DriverParameters & parameters)
{
  std::vector<std::string> errors;
  validate_can_and_control(errors, parameters);
  validate_chassis(errors, parameters.chassis);
  validate_steering(errors, parameters);
  validate_drive_and_degradation(errors, parameters);
  validate_motor_ids(errors, parameters.motors);
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

SteeringModuleConfig steering_module_config(
  const DriverParameters & parameters, std::size_t module_index)
{
  validate_module_index(module_index);
  return SteeringModuleConfig{
    parameters.steering.gear_ratio,
    parameters.steering.inverted[module_index],
    parameters.steering.zero_offset_rad[module_index],
    parameters.steering.kp,
    parameters.steering.kd,
    parameters.steering.kff_omega,
    0.9,
    0.95,
    parameters.steering.max_ff_speed_radps};
}

DriveModuleConfig drive_module_config(
  const DriverParameters & parameters, std::size_t module_index)
{
  validate_module_index(module_index);
  return DriveModuleConfig{
    parameters.drive.gear_ratio,
    parameters.drive.inverted[module_index],
    parameters.chassis.wheel_radius_m,
    parameters.drive.kd,
    parameters.drive.ks,
    parameters.drive.kv,
    parameters.drive.ka,
    parameters.chassis.max_wheel_acceleration_mps2};
}

DmMotorConfig steering_motor_config(
  const DriverParameters & parameters, std::size_t module_index)
{
  validate_module_index(module_index);
  return DmMotorConfig{
    parameters.motors.steering_esc_id[module_index],
    parameters.motors.steering_mst_id[module_index],
    parameters.limits_fallback};
}

DmMotorConfig drive_motor_config(
  const DriverParameters & parameters, std::size_t module_index)
{
  validate_module_index(module_index);
  return DmMotorConfig{
    parameters.motors.drive_esc_id[module_index],
    parameters.motors.drive_mst_id[module_index],
    parameters.limits_fallback};
}

}  // namespace dm_swerve_driver
