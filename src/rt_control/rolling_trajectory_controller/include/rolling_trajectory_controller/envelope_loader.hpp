#ifndef ROLLING_TRAJECTORY_CONTROLLER__ENVELOPE_LOADER_HPP_
#define ROLLING_TRAJECTORY_CONTROLLER__ENVELOPE_LOADER_HPP_

#include <string>

#include "rolling_trajectory_controller/limit_checker.hpp"

namespace rolling_trajectory_controller
{

bool loadProvisionalEnvelope(
  const std::string & path, DynamicEnvelope & envelope,
  std::string & error) noexcept;

}  // namespace rolling_trajectory_controller

#endif  // ROLLING_TRAJECTORY_CONTROLLER__ENVELOPE_LOADER_HPP_
