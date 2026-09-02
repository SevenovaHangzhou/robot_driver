#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include "dm_swerve_driver/dm_frame_codec.hpp"
#include "dm_swerve_driver/feedback_router.hpp"

namespace dm_swerve_driver {
namespace {

constexpr MotorLimits kLimits{12.5, 30.0, 10.0};

[[nodiscard]] ReceivedCanFrame feedback(
  std::uint16_t mst_id, std::uint8_t motor_id)
{
  return ReceivedCanFrame{
    encode_motor_feedback(
      mst_id,
      MotorFeedback{motor_id, MotorError::enabled, 0.0, 0.0, 0.0, 35U, 32U},
      kLimits),
    {}};
}

TEST(FeedbackRouterTest, RoutesByMstIdAndContainsMalformedFrames)
{
  DmMotor first{DmMotorConfig{1U, 0x11U, kLimits}};
  DmMotor second{DmMotorConfig{2U, 0x12U, kLimits}};
  std::array<DmMotor *, kMotorCount> motors{};
  motors[0] = &first;
  motors[1] = &second;

  auto malformed = feedback(0x11U, 2U);
  auto unknown = feedback(0x77U, 7U);
  const std::vector<ReceivedCanFrame> frames{
    feedback(0x12U, 2U), malformed, unknown};
  std::vector<std::string> warnings;

  FeedbackRouteResult result{};
  EXPECT_NO_THROW(result = route_feedback_frames(
      frames, motors,
      [&](const std::string & message) {warnings.push_back(message);}));

  EXPECT_FALSE(result.received[0]);
  EXPECT_TRUE(result.received[1]);
  EXPECT_EQ(result.accepted_frames, 1U);
  EXPECT_EQ(result.rejected_frames, 1U);
  EXPECT_EQ(result.unknown_frames, 1U);
  EXPECT_EQ(second.health().received_frames, 1U);
  EXPECT_EQ(first.health().received_frames, 0U);
  EXPECT_EQ(warnings.size(), 2U);
}

TEST(FeedbackRouterTest, LoggerExceptionsCannotEscapeRealtimePath)
{
  std::array<DmMotor *, kMotorCount> motors{};
  const std::vector<ReceivedCanFrame> frames{feedback(0x77U, 7U)};

  EXPECT_NO_THROW(static_cast<void>(route_feedback_frames(
      frames, motors,
      [](const std::string &) {throw std::runtime_error{"logger failed"};})));
}

}  // namespace
}  // namespace dm_swerve_driver
