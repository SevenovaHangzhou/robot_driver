#include "rolling_trajectory_controller/service_error.hpp"

#include <cstdint>
#include <string>

#include "robot_rt_control_interfaces/msg/rolling_service_result.hpp"
#include "robot_system_interfaces/msg/error_code.hpp"

namespace rolling_trajectory_controller
{

namespace
{

using ErrorCode = robot_system_interfaces::msg::ErrorCode;
using ErrorInfo = robot_system_interfaces::msg::ErrorInfo;
using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;

}  // namespace

std::uint32_t dreeCodeForServiceResult(std::uint8_t result) noexcept
{
  switch (result) {
    case ServiceResult::NONE:
      return ErrorCode::SUCCESS;
    case ServiceResult::WRONG_PROTOCOL:
      return ErrorCode::VERSION_MISMATCH;
    case ServiceResult::WRONG_REQUEST:
    case ServiceResult::WRONG_CLIENT:
    case ServiceResult::AXIS_SET_MISMATCH:
      return ErrorCode::INVALID_GOAL;
    case ServiceResult::WRONG_BOOT:
    case ServiceResult::WRONG_SESSION:
      return ErrorCode::RETRYABLE_INVALID_GOAL;
    case ServiceResult::NOT_ENABLED:
    case ServiceResult::SESSION_BUSY:
    case ServiceResult::SESSION_EXISTS:
    case ServiceResult::SOURCE_STATE_STALE:
    case ServiceResult::SOURCE_MOVING:
    case ServiceResult::FEEDBACK_STALE:
    case ServiceResult::SWITCH_TIMEOUT:
    case ServiceResult::NOT_READY:
      return ErrorCode::NOT_READY;
    case ServiceResult::WRONG_MODE:
    case ServiceResult::SWITCH_REJECTED:
      return ErrorCode::GOAL_REJECTED;
    case ServiceResult::TAKEOVER_MISMATCH:
      return ErrorCode::RT_TOLERANCE_VIOLATED;
    case ServiceResult::UNSAFE_HOLD:
      return ErrorCode::SAFETY_DENIED;
    case ServiceResult::LIMITS_UNAVAILABLE:
    case ServiceResult::RESTART_REQUIRED:
      return ErrorCode::VERSION_MISMATCH;
    default:
      return ErrorCode::INTERNAL_ERROR;
  }
}

void populateServiceError(ErrorInfo & error, std::uint8_t result)
{
  const std::uint32_t code = dreeCodeForServiceResult(result);
  error.code = code;
  error.retryable = code != ErrorCode::SUCCESS && ((code / 100U) % 10U) == 1U;
  error.severity = code == ErrorCode::SUCCESS ?
    ErrorInfo::OK : (error.retryable ? ErrorInfo::WARN : ErrorInfo::FAULT);
  error.source = "rt_control";
  error.message = code == ErrorCode::SUCCESS ?
    "rolling request accepted" : "rolling request rejected";
  error.detail = "rolling_service_result=" + std::to_string(result);
}

}  // namespace rolling_trajectory_controller
