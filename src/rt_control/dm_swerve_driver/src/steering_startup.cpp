#include "steering_startup.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace dm_swerve_driver {
namespace {

void log_safely(
  const StartupLogger & log, DriverLogLevel level, const std::string & message) noexcept
{
  try {
    if (log) {
      log(level, message);
    }
  } catch (...) {
  }
}

[[nodiscard]] double steering_direction(
  const DriverParameters & parameters, std::size_t index) noexcept
{
  return parameters.steering.inverted[index] ? -1.0 : 1.0;
}

[[nodiscard]] double motor_position_for_steering_angle(
  const DriverParameters & parameters, std::size_t index, double angle_rad) noexcept
{
  return (angle_rad + parameters.steering.zero_offset_rad[index]) *
         parameters.steering.gear_ratio * steering_direction(parameters, index);
}

}  // namespace

bool validate_steering_command_ranges(
  const DriverParameters & parameters,
  const std::array<DmMotor *, kMotorCount> & motors,
  const StartupLogger & log)
{
  const double safe_minimum = parameters.steering.joint_limit_min_rad +
    parameters.steering.joint_limit_margin_rad;
  const double safe_maximum = parameters.steering.joint_limit_max_rad -
    parameters.steering.joint_limit_margin_rad;
  bool valid{true};
  for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
    const double lower = motor_position_for_steering_angle(
      parameters, index, safe_minimum);
    const double upper = motor_position_for_steering_angle(
      parameters, index, safe_maximum);
    if (std::max(std::abs(lower), std::abs(upper)) <= motors[index]->limits().position_max) {
      continue;
    }
    std::ostringstream message;
    message << "ESC_ID " << motors[index]->esc_id() <<
      " mechanical steering range cannot be represented by motor PMAX; "
      "strict startup rejected";
    log_safely(log, DriverLogLevel::error, message.str());
    valid = false;
  }
  return valid;
}

bool validate_seeded_steering_positions(
  const DriverParameters & parameters,
  const std::array<DmMotor *, kMotorCount> & motors,
  const StartupLogger & log)
{
  const SteeringAngleLimits limits{
    parameters.steering.joint_limit_min_rad,
    parameters.steering.joint_limit_max_rad,
    parameters.steering.joint_limit_margin_rad,
    parameters.steering.joint_limit_tolerance_rad};
  bool valid{true};
  for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
    const double angle = motors[index]->unwrapped_position() /
      (parameters.steering.gear_ratio * steering_direction(parameters, index)) -
      parameters.steering.zero_offset_rad[index];
    if (steering_measurement_within_tolerance(angle, limits)) {
      continue;
    }
    std::ostringstream message;
    message << "ESC_ID " << motors[index]->esc_id() <<
      " seeded steering angle is outside mechanical limits; strict startup rejected";
    log_safely(log, DriverLogLevel::error, message.str());
    valid = false;
  }
  return valid;
}

}  // namespace dm_swerve_driver
