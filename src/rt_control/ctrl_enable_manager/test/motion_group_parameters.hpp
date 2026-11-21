#ifndef CTRL_ENABLE_MANAGER__TEST__MOTION_GROUP_PARAMETERS_HPP_
#define CTRL_ENABLE_MANAGER__TEST__MOTION_GROUP_PARAMETERS_HPP_

#include <set>
#include <string>
#include <vector>

#include "rclcpp/parameter.hpp"

namespace ctrl_enable_manager::test
{

// The enable manager reads motion-group parameters as <group>.<name>. The fixtures list them by
// name for a single group; these helpers move them into that group and name the group in
// motion_groups unless its motion controller is empty (an enable-only or companion-only manager).
inline bool isGroupParameter(const std::string & name)
{
  static const std::set<std::string> names{
    "motion_controller",
    "rolling_controller",
    "trajectory_controller",
    "mode_service",
    "mode_switch_joints",
    "motion_joints",
    "mode_switch_maximum_sample_period_ms",
    "mode_switch_source_state_max_age_ms",
    "mode_switch_stable_interval_count",
    "mode_switch_timeout_ms",
    "mode_switch_feedback_age_limit_ms",
    "mode_switch_stable_velocity_thresholds",
    "mode_switch_takeover_tolerances"};
  return names.count(name) != 0U;
}

inline rclcpp::Parameter groupedParameter(
  const rclcpp::Parameter & parameter, const std::string & group)
{
  return isGroupParameter(parameter.get_name())
           ? rclcpp::Parameter(group + "." + parameter.get_name(), parameter.get_parameter_value())
           : parameter;
}

inline std::vector<rclcpp::Parameter> grouped(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & group)
{
  std::vector<rclcpp::Parameter> result;
  bool has_controller = false;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "motion_controller") {
      has_controller = !parameter.as_string().empty();
    }
    result.push_back(groupedParameter(parameter, group));
  }
  if (has_controller) {
    result.emplace_back("motion_groups", std::vector<std::string>{group});
  }
  return result;
}

}  // namespace ctrl_enable_manager::test

#endif  // CTRL_ENABLE_MANAGER__TEST__MOTION_GROUP_PARAMETERS_HPP_
