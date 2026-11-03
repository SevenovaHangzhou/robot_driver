#ifndef CTRL_ROLLING_TRAJECTORY__AXIS_SET_HPP_
#define CTRL_ROLLING_TRAJECTORY__AXIS_SET_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "ctrl_rolling_trajectory/rolling_types.hpp"

namespace ctrl_rolling_trajectory
{

// Joint names come from the machine configuration; the controller owns only the axis count.
using JointNames = std::array<std::string, kAxisCount>;
using AxisSetHash = std::array<std::uint8_t, 32>;

// Accepts exactly kAxisCount distinct, non-empty names in protocol axis order.
bool parseJointNames(
  const std::vector<std::string> & values, JointNames & names, std::string & error);

// Parses a SHA-256 digest given as 64 lowercase hex digits. The composer computes the
// protocol axis-set identity and the envelope limits_version; the controller only compares.
bool parseSha256Hex(const std::string & text, AxisSetHash & digest, std::string & error);

}  // namespace ctrl_rolling_trajectory

#endif  // CTRL_ROLLING_TRAJECTORY__AXIS_SET_HPP_
