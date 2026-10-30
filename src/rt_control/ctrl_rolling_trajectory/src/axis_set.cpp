#include "ctrl_rolling_trajectory/axis_set.hpp"

#include <openssl/sha.h>

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

AxisSetHash computeAxisSetHash(const JointNames & names)
{
  std::string text;
  for (const std::string & name : names) {
    text += name + ":rad:rad/s\n";
  }
  AxisSetHash hash{};
  SHA256(reinterpret_cast<const unsigned char *>(text.data()), text.size(), hash.data());
  return hash;
}

}  // namespace ctrl_rolling_trajectory
