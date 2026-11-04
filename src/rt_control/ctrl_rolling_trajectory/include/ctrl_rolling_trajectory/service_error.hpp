#ifndef CTRL_ROLLING_TRAJECTORY__SERVICE_ERROR_HPP_
#define CTRL_ROLLING_TRAJECTORY__SERVICE_ERROR_HPP_

#include <cstdint>

#include "robot_system_interfaces/msg/error_info.hpp"

namespace ctrl_rolling_trajectory
{

[[nodiscard]] std::uint32_t dreeCodeForServiceResult(std::uint8_t result) noexcept;

void populateServiceError(robot_system_interfaces::msg::ErrorInfo & error, std::uint8_t result);

}  // namespace ctrl_rolling_trajectory

#endif  // CTRL_ROLLING_TRAJECTORY__SERVICE_ERROR_HPP_
