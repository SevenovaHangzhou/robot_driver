#include "ctrl_rolling_trajectory/axis_set.hpp"

#include <set>

namespace ctrl_rolling_trajectory
{

bool parseJointNames(
  const std::vector<std::string> & values, JointNames & names, std::string & error)
{
  if (values.size() != kAxisCount) {
    error = "joints must contain exactly 14 names in protocol axis order";
    return false;
  }
  if (std::set<std::string>(values.begin(), values.end()).size() != kAxisCount) {
    error = "joints must be distinct";
    return false;
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (values[axis].empty()) {
      error = "joints must not contain an empty name";
      return false;
    }
    names[axis] = values[axis];
  }
  return true;
}

bool parseSha256Hex(const std::string & text, AxisSetHash & digest, std::string & error)
{
  if (text.size() != 2U * digest.size()) {
    error = "must be 64 lowercase hex digits";
    return false;
  }
  for (std::size_t index = 0U; index < digest.size(); ++index) {
    std::uint8_t byte = 0U;
    for (const char character : text.substr(2U * index, 2U)) {
      byte = static_cast<std::uint8_t>(byte << 4U);
      if (character >= '0' && character <= '9') {
        byte = static_cast<std::uint8_t>(byte | static_cast<std::uint8_t>(character - '0'));
      } else if (character >= 'a' && character <= 'f') {
        byte = static_cast<std::uint8_t>(byte | static_cast<std::uint8_t>(character - 'a' + 10));
      } else {
        error = "must be 64 lowercase hex digits";
        return false;
      }
    }
    digest[index] = byte;
  }
  return true;
}

}  // namespace ctrl_rolling_trajectory
