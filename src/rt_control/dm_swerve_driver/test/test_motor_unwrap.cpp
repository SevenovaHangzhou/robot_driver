#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "dm_swerve_driver/dm_frame_codec.hpp"
#include "dm_swerve_driver/dm_motor.hpp"
#include "dm_swerve_driver/swerve_kinematics.hpp"

namespace dm_swerve_driver {
namespace {

constexpr MotorLimits kLimits{10.0, 30.0, 8.0};
constexpr double kPositionTolerance{2.0 * kLimits.position_max / 65535.0};

[[nodiscard]] CanFrame feedback_frame(
  double position, MotorError error = MotorError::enabled,
  double velocity = 0.0, std::uint8_t motor_id = 3U,
  std::uint16_t mst_id = 0x13U)
{
  return encode_motor_feedback(
    mst_id, MotorFeedback{motor_id, error, position, velocity, 0.0, 40U, 35U}, kLimits);
}

TEST(DmMotorTest, UnwrapsPositiveAndNegativeBoundaryCrossings)
{
  DmMotor positive{DmMotorConfig{3U, 0x13U, kLimits}};
  static_cast<void>(positive.accept_feedback(feedback_frame(9.5)));
  static_cast<void>(positive.accept_feedback(feedback_frame(-9.5)));
  EXPECT_NEAR(positive.unwrapped_position(), 10.5, kPositionTolerance);

  DmMotor negative{DmMotorConfig{3U, 0x13U, kLimits}};
  static_cast<void>(negative.accept_feedback(feedback_frame(-9.5)));
  static_cast<void>(negative.accept_feedback(feedback_frame(9.5)));
  EXPECT_NEAR(negative.unwrapped_position(), -10.5, kPositionTolerance);
}

TEST(DmMotorTest, OrdinaryPositionChangesDoNotChangeWrapOffset)
{
  DmMotor motor{DmMotorConfig{3U, 0x13U, kLimits}};
  static_cast<void>(motor.accept_feedback(feedback_frame(2.0)));
  static_cast<void>(motor.accept_feedback(feedback_frame(4.0)));
  EXPECT_NEAR(motor.unwrapped_position(), 4.0, kPositionTolerance);
}

TEST(DmMotorTest, MultiTurnPositionSeedsUnwrapperAndChecksPrincipalValue)
{
  DmMotor motor{DmMotorConfig{3U, 0x13U, kLimits}};
  EXPECT_FALSE(motor.seed_position_from_multi_turn(1.0, 41.0, 0.1));
  EXPECT_DOUBLE_EQ(motor.unwrapped_position(), 41.0);
  EXPECT_TRUE(motor.health().seeded_from_multi_turn);

  EXPECT_TRUE(DmMotor::multi_turn_consistent(0.02, 2.0 * kPi + 0.01, 0.1));
  EXPECT_FALSE(DmMotor::multi_turn_consistent(0.02, 2.0 * kPi + 0.3, 0.1));
  EXPECT_TRUE(DmMotor::multi_turn_consistent(
      kPi - 0.02, -kPi + 0.02, 0.1));
}

TEST(DmMotorTest, HealthCountsMissesAndRecoversOnFeedback)
{
  DmMotor motor{DmMotorConfig{3U, 0x13U, kLimits}};
  motor.mark_feedback_missed();
  motor.mark_feedback_missed();
  EXPECT_FALSE(motor.health().has_feedback);
  EXPECT_EQ(motor.health().missed_frames, 2U);
  EXPECT_EQ(motor.health().consecutive_missed_frames, 2U);

  static_cast<void>(motor.accept_feedback(feedback_frame(0.0)));
  EXPECT_TRUE(motor.health().has_feedback);
  EXPECT_FALSE(motor.health().seeded_from_multi_turn);
  EXPECT_TRUE(motor.health().enabled());
  EXPECT_FALSE(motor.health().has_fault());
  EXPECT_EQ(motor.health().received_frames, 1U);
  EXPECT_EQ(motor.health().consecutive_missed_frames, 0U);
  EXPECT_EQ(motor.health().mos_temperature_c, 40U);

  static_cast<void>(motor.accept_feedback(
      feedback_frame(0.0, MotorError::over_current)));
  EXPECT_TRUE(motor.health().has_fault());
  EXPECT_FALSE(motor.health().enabled());
}

TEST(DmMotorTest, RejectsFeedbackForWrongCanOrMotorIdentifier)
{
  DmMotor motor{DmMotorConfig{3U, 0x13U, kLimits}};
  EXPECT_THROW(
    static_cast<void>(motor.accept_feedback(
        feedback_frame(0.0, MotorError::enabled, 0.0, 3U, 0x14U))),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(motor.accept_feedback(
        feedback_frame(0.0, MotorError::enabled, 0.0, 4U, 0x13U))),
    std::invalid_argument);
  EXPECT_EQ(motor.health().received_frames, 0U);
}

TEST(DmMotorTest, BuildsCommandsWithOwnedIdentifiersAndLimits)
{
  DmMotor motor{DmMotorConfig{3U, 0x13U, kLimits}};
  const auto command = motor.encode_command(MitCommand{1.0, 2.0, 3.0, 1.0, 0.5});
  EXPECT_EQ(command.id, 3U);
  const auto decoded = decode_mit_command(command, kLimits);
  EXPECT_NEAR(decoded.position, 1.0, kPositionTolerance);
  EXPECT_EQ(motor.special_command(SpecialCommand::disable).id, 3U);
}

TEST(DmMotorTest, LimitsCanOnlyChangeBeforePositionInitialization)
{
  DmMotor motor{DmMotorConfig{3U, 0x13U, kLimits}};
  motor.set_limits(MotorLimits{20.0, 40.0, 12.0});
  EXPECT_DOUBLE_EQ(motor.limits().position_max, 20.0);
  motor.seed_position(0.0);
  EXPECT_THROW(motor.set_limits(kLimits), std::logic_error);
}

TEST(DmMotorTest, RejectsInvalidConfigurationAndSeeds)
{
  EXPECT_THROW(
    DmMotor(DmMotorConfig{0x800U, 1U, kLimits}), std::invalid_argument);
  EXPECT_THROW(
    DmMotor(DmMotorConfig{1U, 1U, MotorLimits{}}), std::invalid_argument);
  DmMotor motor{DmMotorConfig{3U, 0x13U, kLimits}};
  EXPECT_THROW(motor.seed_position(11.0), std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(motor.seed_position_from_multi_turn(0.0, 0.0, -0.1)),
    std::invalid_argument);
}

}  // namespace
}  // namespace dm_swerve_driver
