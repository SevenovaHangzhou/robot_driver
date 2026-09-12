#ifndef DM_SWERVE_DRIVER__SRC__STEERING_STARTUP_HPP_
#define DM_SWERVE_DRIVER__SRC__STEERING_STARTUP_HPP_

#include <array>

#include "motor_startup.hpp"

namespace dm_swerve_driver {

[[nodiscard]] bool validate_steering_command_ranges(
  const DriverParameters & parameters,
  const std::array<DmMotor *, kMotorCount> & motors,
  const StartupLogger & log);

[[nodiscard]] bool validate_seeded_steering_positions(
  const DriverParameters & parameters,
  const std::array<DmMotor *, kMotorCount> & motors,
  const StartupLogger & log);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__SRC__STEERING_STARTUP_HPP_
