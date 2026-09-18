#ifndef ROLLING_TRAJECTORY_CONTROLLER__SERVICE_ERROR_HPP_
#define ROLLING_TRAJECTORY_CONTROLLER__SERVICE_ERROR_HPP_

#include <cstdint>

#include "robot_system_interfaces/msg/error_info.hpp"

namespace rolling_trajectory_controller
{

[[nodiscard]] std::uint32_t dreeCodeForServiceResult(std::uint8_t result) noexcept;

void populateServiceError(
  robot_system_interfaces::msg::ErrorInfo & error, std::uint8_t result);

}  // namespace rolling_trajectory_controller

#endif  // ROLLING_TRAJECTORY_CONTROLLER__SERVICE_ERROR_HPP_
