#ifndef RT_DIAGNOSTICS__DYNAMIC_STATE_SNAPSHOT_HPP_
#define RT_DIAGNOSTICS__DYNAMIC_STATE_SNAPSHOT_HPP_

#include <string>
#include <unordered_map>

#include "control_msgs/msg/dynamic_joint_state.hpp"

namespace rt_diagnostics
{

bool replaceDynamicStateSnapshot(
  const control_msgs::msg::DynamicJointState & message,
  std::unordered_map<std::string, double> & values);

}  // namespace rt_diagnostics

#endif  // RT_DIAGNOSTICS__DYNAMIC_STATE_SNAPSHOT_HPP_
