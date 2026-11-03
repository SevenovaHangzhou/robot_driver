#ifndef CTRL_ROLLING_TRAJECTORY__ENVELOPE_LOADER_HPP_
#define CTRL_ROLLING_TRAJECTORY__ENVELOPE_LOADER_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "ctrl_rolling_trajectory/axis_set.hpp"
#include "ctrl_rolling_trajectory/limit_checker.hpp"

namespace ctrl_rolling_trajectory
{

// Axis names in the file must equal `joints`, in order. `limits_version` is the digest the
// composer computed over the artifact copy at `path`; the loader does not hash.
bool loadProvisionalEnvelope(
  const std::string & path, const JointNames & joints,
  const std::array<std::uint8_t, 32> & limits_version, DynamicEnvelope & envelope,
  std::string & error) noexcept;

}  // namespace ctrl_rolling_trajectory

#endif  // CTRL_ROLLING_TRAJECTORY__ENVELOPE_LOADER_HPP_
