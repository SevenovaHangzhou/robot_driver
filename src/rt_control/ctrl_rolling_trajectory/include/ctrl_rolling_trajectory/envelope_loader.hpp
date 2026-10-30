#ifndef CTRL_ROLLING_TRAJECTORY__ENVELOPE_LOADER_HPP_
#define CTRL_ROLLING_TRAJECTORY__ENVELOPE_LOADER_HPP_

#include <string>

#include "ctrl_rolling_trajectory/axis_set.hpp"
#include "ctrl_rolling_trajectory/limit_checker.hpp"

namespace ctrl_rolling_trajectory
{

// Axis names in the file must equal `joints`, in order.
bool loadProvisionalEnvelope(
  const std::string & path, const JointNames & joints, DynamicEnvelope & envelope,
  std::string & error) noexcept;

}  // namespace ctrl_rolling_trajectory

#endif  // CTRL_ROLLING_TRAJECTORY__ENVELOPE_LOADER_HPP_
