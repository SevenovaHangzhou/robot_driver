#ifndef DM_SWERVE_DRIVER__SRC__MOTOR_STARTUP_HPP_
#define DM_SWERVE_DRIVER__SRC__MOTOR_STARTUP_HPP_

#include <array>
#include <functional>
#include <string>

#include "dm_swerve_driver/control_loop.hpp"

namespace dm_swerve_driver {

using StartupLogger = std::function<void(DriverLogLevel, const std::string &)>;

void initialize_motors(
  const DriverParameters & parameters,
  CanTransport & transport,
  const std::array<DmMotor *, kMotorCount> & motors,
  const StartupLogger & log);

void disable_motors(
  CanTransport & transport,
  const std::array<DmMotor *, kMotorCount> & motors,
  const StartupLogger & log) noexcept;

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__SRC__MOTOR_STARTUP_HPP_
