#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>

#include "robot_rt_control_interfaces/msg/rolling_service_result.hpp"
#include "robot_system_interfaces/msg/error_code.hpp"
#include "robot_system_interfaces/msg/error_info.hpp"
#include "rolling_trajectory_controller/service_error.hpp"

namespace rolling_trajectory_controller
{
namespace
{

using ErrorCode = robot_system_interfaces::msg::ErrorCode;
using ErrorInfo = robot_system_interfaces::msg::ErrorInfo;
using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;

struct MappingCase
{
  std::uint8_t result;
  std::uint32_t code;
  bool retryable;
};

TEST(ServiceError, MapsEveryPublicResultToStableDreeEnvelope)
{
  constexpr std::array<MappingCase, 21U> cases{{
    {ServiceResult::NONE, ErrorCode::SUCCESS, false},
    {ServiceResult::WRONG_PROTOCOL, ErrorCode::VERSION_MISMATCH, false},
    {ServiceResult::WRONG_REQUEST, ErrorCode::INVALID_GOAL, false},
    {ServiceResult::WRONG_MODE, ErrorCode::GOAL_REJECTED, false},
    {ServiceResult::NOT_ENABLED, ErrorCode::NOT_READY, true},
    {ServiceResult::SESSION_BUSY, ErrorCode::NOT_READY, true},
    {ServiceResult::SESSION_EXISTS, ErrorCode::NOT_READY, true},
    {ServiceResult::WRONG_BOOT, ErrorCode::RETRYABLE_INVALID_GOAL, true},
    {ServiceResult::WRONG_SESSION, ErrorCode::RETRYABLE_INVALID_GOAL, true},
    {ServiceResult::WRONG_CLIENT, ErrorCode::INVALID_GOAL, false},
    {ServiceResult::AXIS_SET_MISMATCH, ErrorCode::INVALID_GOAL, false},
    {ServiceResult::SOURCE_STATE_STALE, ErrorCode::NOT_READY, true},
    {ServiceResult::SOURCE_MOVING, ErrorCode::NOT_READY, true},
    {ServiceResult::TAKEOVER_MISMATCH, ErrorCode::RT_TOLERANCE_VIOLATED, false},
    {ServiceResult::UNSAFE_HOLD, ErrorCode::SAFETY_DENIED, false},
    {ServiceResult::FEEDBACK_STALE, ErrorCode::NOT_READY, true},
    {ServiceResult::LIMITS_UNAVAILABLE, ErrorCode::VERSION_MISMATCH, false},
    {ServiceResult::SWITCH_REJECTED, ErrorCode::GOAL_REJECTED, false},
    {ServiceResult::SWITCH_TIMEOUT, ErrorCode::NOT_READY, true},
    {ServiceResult::RESTART_REQUIRED, ErrorCode::VERSION_MISMATCH, false},
    {ServiceResult::NOT_READY, ErrorCode::NOT_READY, true},
  }};

  for (const MappingCase & test_case : cases) {
    ErrorInfo error;
    populateServiceError(error, test_case.result);
    EXPECT_EQ(error.code, test_case.code) << "result=" << +test_case.result;
    EXPECT_EQ(error.retryable, test_case.retryable) << "result=" << +test_case.result;
    EXPECT_EQ(
      error.severity,
      test_case.code == ErrorCode::SUCCESS ? ErrorInfo::OK :
      (test_case.retryable ? ErrorInfo::WARN : ErrorInfo::FAULT));
    EXPECT_EQ(error.source, "rt_control");
    EXPECT_FALSE(error.message.empty());
    EXPECT_EQ(error.detail, "rolling_service_result=" + std::to_string(test_case.result));
  }
}

TEST(ServiceError, UnknownResultFailsClosedAsInternalError)
{
  ErrorInfo error;
  populateServiceError(error, 255U);

  EXPECT_EQ(error.code, ErrorCode::INTERNAL_ERROR);
  EXPECT_FALSE(error.retryable);
  EXPECT_EQ(error.severity, ErrorInfo::FAULT);
}

}  // namespace
}  // namespace rolling_trajectory_controller
