#include "rt_diagnostics/dynamic_state_snapshot.hpp"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace rt_diagnostics
{

bool replaceDynamicStateSnapshot(
  const control_msgs::msg::DynamicJointState & message,
  std::unordered_map<std::string, double> & values)
{
  std::unordered_map<std::string, double> candidate;
  if (message.joint_names.size() != message.interface_values.size()) {
    values.clear();
    return false;
  }

  std::unordered_set<std::string> joint_names;
  joint_names.reserve(message.joint_names.size());
  for (std::size_t joint = 0; joint < message.joint_names.size(); ++joint) {
    const auto & joint_name = message.joint_names[joint];
    const auto & interfaces = message.interface_values[joint];
    if (
      joint_name.empty() || !joint_names.insert(joint_name).second ||
      interfaces.interface_names.size() != interfaces.values.size())
    {
      values.clear();
      return false;
    }

    std::unordered_set<std::string> interface_names;
    interface_names.reserve(interfaces.interface_names.size());
    for (std::size_t item = 0; item < interfaces.interface_names.size(); ++item) {
      const auto & interface_name = interfaces.interface_names[item];
      if (interface_name.empty() || !interface_names.insert(interface_name).second) {
        values.clear();
        return false;
      }
      candidate.emplace(
        joint_name + "/" + interface_name, interfaces.values[item]);
    }
  }

  values = std::move(candidate);
  return true;
}

}  // namespace rt_diagnostics
