#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include "dm_swerve_driver/dm_frame_codec.hpp"
#include "dm_swerve_driver/fake_motor_model.hpp"

namespace dm_swerve_driver {
namespace {

using namespace std::chrono_literals;

constexpr MotorLimits kLimits{12.5, 30.0, 10.0};

TEST(FakeMotorModelTest, EnableAndMitCommandProduceFeedback)
{
  FakeMotorModel motor{FakeMotorConfig{1U, 0x11U, kLimits, 30ms}};

  const auto disabled_reply = motor.handle_frame(
    encode_mit_command(1U, MitCommand{}, kLimits), 10ms);
  ASSERT_TRUE(disabled_reply.has_value());
  EXPECT_EQ(decode_motor_feedback(*disabled_reply, kLimits).error, MotorError::disabled);

  const auto enable_reply = motor.handle_frame(
    make_special_command(1U, SpecialCommand::enable), 10ms);
  ASSERT_TRUE(enable_reply.has_value());
  EXPECT_TRUE(motor.enabled());
  EXPECT_EQ(decode_motor_feedback(*enable_reply, kLimits).error, MotorError::enabled);

  const auto command_reply = motor.handle_frame(
    encode_mit_command(1U, MitCommand{0.0, 6.0, 0.0, 2.0, 1.0}, kLimits), 10ms);
  ASSERT_TRUE(command_reply.has_value());
  EXPECT_GT(motor.velocity(), 0.0);
  EXPECT_GT(motor.position(), 0.0);
}

TEST(FakeMotorModelTest, IgnoresFramesForOtherMotors)
{
  FakeMotorModel motor{FakeMotorConfig{1U, 0x11U, kLimits, 30ms}};
  EXPECT_FALSE(motor.handle_frame(make_special_command(2U, SpecialCommand::enable), 10ms));
}

TEST(FakeMotorModelTest, ReadsLimitAndEchoesWritableRegister)
{
  FakeMotorModel motor{FakeMotorConfig{0x345U, 0x22U, kLimits, 30ms}};

  const auto limit_reply = motor.handle_frame(
    make_register_read(0x345U, static_cast<std::uint8_t>(RegisterId::position_max)), 1ms);
  ASSERT_TRUE(limit_reply.has_value());
  const auto decoded_limit = decode_register_reply(*limit_reply);
  EXPECT_EQ(decoded_limit.operation, RegisterOperation::read);
  EXPECT_FLOAT_EQ(decoded_limit.float_value(), 12.5F);

  const auto timeout_reply = motor.handle_frame(
    make_register_write(
      0x345U, static_cast<std::uint8_t>(RegisterId::timeout), 2000U),
    1ms);
  ASSERT_TRUE(timeout_reply.has_value());
  const auto decoded_timeout = decode_register_reply(*timeout_reply);
  EXPECT_EQ(decoded_timeout.operation, RegisterOperation::write);
  EXPECT_EQ(decoded_timeout.raw_value, 2000U);
}

TEST(FakeMotorModelTest, SaveZeroResetsReportedPosition)
{
  FakeMotorModel motor{FakeMotorConfig{1U, 0x11U, kLimits, 30ms}};
  static_cast<void>(motor.handle_frame(make_special_command(1U, SpecialCommand::enable), 1ms));
  static_cast<void>(motor.handle_frame(
      encode_mit_command(1U, MitCommand{0.0, 10.0, 0.0, 1.0, 0.0}, kLimits), 100ms));
  ASSERT_NE(motor.position(), 0.0);

  const auto reply = motor.handle_frame(
    make_special_command(1U, SpecialCommand::save_zero), 1ms);
  ASSERT_TRUE(reply.has_value());
  EXPECT_DOUBLE_EQ(motor.position(), 0.0);
}

TEST(FakeMotorModelTest, DisableStopsMotorAndClearFaultPreservesDisabledState)
{
  FakeMotorModel motor{FakeMotorConfig{1U, 0x11U, kLimits, 30ms}};
  EXPECT_EQ(motor.esc_id(), 1U);
  EXPECT_EQ(motor.mst_id(), 0x11U);
  static_cast<void>(motor.handle_frame(make_special_command(1U, SpecialCommand::enable), 1ms));
  static_cast<void>(motor.handle_frame(
      encode_mit_command(1U, MitCommand{0.0, 10.0, 0.0, 1.0, 0.0}, kLimits), 100ms));
  ASSERT_GT(motor.velocity(), 0.0);

  const auto disabled = motor.handle_frame(
    make_special_command(1U, SpecialCommand::disable), 1ms);
  ASSERT_TRUE(disabled.has_value());
  EXPECT_FALSE(motor.enabled());
  EXPECT_DOUBLE_EQ(motor.velocity(), 0.0);

  const auto cleared = motor.handle_frame(
    make_special_command(1U, SpecialCommand::clear_fault), 1ms);
  ASSERT_TRUE(cleared.has_value());
  EXPECT_EQ(decode_motor_feedback(*cleared, kLimits).error, MotorError::disabled);
}

TEST(FakeMotorModelTest, ReadsAllStartupRegistersAndIgnoresOtherTargets)
{
  FakeMotorModel motor{FakeMotorConfig{0x345U, 0x22U, kLimits, 30ms}};
  const auto read_float = [&](RegisterId register_id) {
      const auto response = motor.handle_frame(
        make_register_read(0x345U, static_cast<std::uint8_t>(register_id)), 1ms);
      EXPECT_TRUE(response.has_value());
      return decode_register_reply(*response).float_value();
    };

  EXPECT_FLOAT_EQ(read_float(RegisterId::velocity_max), 30.0F);
  EXPECT_FLOAT_EQ(read_float(RegisterId::torque_max), 10.0F);
  EXPECT_FLOAT_EQ(read_float(RegisterId::multi_turn_position), 0.0F);

  static_cast<void>(motor.handle_frame(
      make_register_write(
        0x345U, static_cast<std::uint8_t>(RegisterId::timeout), 1234U),
      1ms));
  const auto timeout = motor.handle_frame(
    make_register_read(0x345U, static_cast<std::uint8_t>(RegisterId::timeout)), 1ms);
  ASSERT_TRUE(timeout.has_value());
  EXPECT_EQ(decode_register_reply(*timeout).raw_value, 1234U);

  const auto unknown = motor.handle_frame(make_register_read(0x345U, 0x7EU), 1ms);
  ASSERT_TRUE(unknown.has_value());
  EXPECT_EQ(decode_register_reply(*unknown).raw_value, 0U);
  EXPECT_FALSE(motor.handle_frame(make_register_read(0x344U, 0x15U), 1ms));
}

TEST(FakeMotorModelTest, NegativeMotionWrapsPrincipalFeedbackPosition)
{
  FakeMotorModel motor{FakeMotorConfig{1U, 0x11U, kLimits, 30ms}};
  static_cast<void>(motor.handle_frame(make_special_command(1U, SpecialCommand::enable), 1ms));
  const auto response = motor.handle_frame(
    encode_mit_command(1U, MitCommand{0.0, -30.0, 0.0, 0.0, 0.0}, kLimits), 1s);
  ASSERT_TRUE(response.has_value());
  const auto feedback = decode_motor_feedback(*response, kLimits);
  EXPECT_GE(feedback.position, -kLimits.position_max);
  EXPECT_LE(feedback.position, kLimits.position_max);
  EXPECT_LT(motor.position(), -kLimits.position_max);
}

TEST(FakeMotorModelTest, RejectsInvalidConfiguration)
{
  EXPECT_THROW(
    FakeMotorModel(FakeMotorConfig{0x800U, 1U, kLimits, 30ms}),
    std::invalid_argument);
  EXPECT_THROW(
    FakeMotorModel(FakeMotorConfig{1U, 1U, MotorLimits{}, 30ms}),
    std::invalid_argument);
  EXPECT_THROW(
    FakeMotorModel(FakeMotorConfig{1U, 1U, kLimits, 0ms}),
    std::invalid_argument);
}

}  // namespace
}  // namespace dm_swerve_driver
