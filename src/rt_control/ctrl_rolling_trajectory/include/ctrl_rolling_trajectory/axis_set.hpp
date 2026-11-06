#ifndef CTRL_ROLLING_TRAJECTORY__AXIS_SET_HPP_
#define CTRL_ROLLING_TRAJECTORY__AXIS_SET_HPP_

#include <array>
#include <string>
#include <vector>

#include "ctrl_rolling_trajectory/rolling_types.hpp"

namespace ctrl_rolling_trajectory
{

// Joint names come from the machine configuration; the controller owns only the axis count.
using JointNames = std::array<std::string, kAxisCount>;

// Accepts exactly kAxisCount distinct, non-empty names in protocol axis order.
bool parseJointNames(
  const std::vector<std::string> & values, JointNames & names, std::string & error);

}  // namespace ctrl_rolling_trajectory

#endif  // CTRL_ROLLING_TRAJECTORY__AXIS_SET_HPP_
