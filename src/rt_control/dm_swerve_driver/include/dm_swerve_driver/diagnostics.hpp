#ifndef DM_SWERVE_DRIVER__DIAGNOSTICS_HPP_
#define DM_SWERVE_DRIVER__DIAGNOSTICS_HPP_

#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "dm_swerve_driver/control_loop.hpp"
#include "dm_swerve_driver/params.hpp"

namespace dm_swerve_driver {

[[nodiscard]] std::vector<diagnostic_msgs::msg::DiagnosticStatus>
build_diagnostic_statuses(
  const ControlLoopStatus & status,
  const DriverParameters & parameters);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__DIAGNOSTICS_HPP_
