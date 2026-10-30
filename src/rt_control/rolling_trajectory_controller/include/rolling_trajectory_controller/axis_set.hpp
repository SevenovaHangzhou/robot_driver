#ifndef ROLLING_TRAJECTORY_CONTROLLER__AXIS_SET_HPP_
#define ROLLING_TRAJECTORY_CONTROLLER__AXIS_SET_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "rolling_trajectory_controller/rolling_types.hpp"

namespace rolling_trajectory_controller
{

// Joint names come from the machine configuration; the controller owns only the axis count.
using JointNames = std::array<std::string, kAxisCount>;
using AxisSetHash = std::array<std::uint8_t, 32>;

// Accepts exactly kAxisCount distinct, non-empty names in protocol axis order.
bool parseJointNames(
  const std::vector<std::string> & values, JointNames & names, std::string & error);

// Protocol axis-set identity: SHA-256 of "<joint_name>:rad:rad/s\n" per axis, in order.
AxisSetHash computeAxisSetHash(const JointNames & names);

}  // namespace rolling_trajectory_controller

#endif  // ROLLING_TRAJECTORY_CONTROLLER__AXIS_SET_HPP_
