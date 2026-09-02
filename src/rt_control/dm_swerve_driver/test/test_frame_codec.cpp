#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "dm_swerve_driver/dm_frame_codec.hpp"

namespace dm_swerve_driver {
namespace {

constexpr MotorLimits kLimits{12.5, 30.0, 10.0};

TEST(FrameCodecTest, FloatMappingClampsAndRoundTrips)
{
  EXPECT_EQ(float_to_uint(-100.0, -10.0, 10.0, 12U), 0U);
  EXPECT_EQ(float_to_uint(100.0, -10.0, 10.0, 12U), 4095U);
  EXPECT_EQ(float_to_uint(0.0, -10.0, 10.0, 12U), 2047U);

  constexpr double one_lsb{20.0 / 4095.0};
  for (const double value : {-10.0, -3.25, 0.0, 7.75, 10.0}) {
    const auto encoded = float_to_uint(value, -10.0, 10.0, 12U);
    EXPECT_NEAR(uint_to_float(encoded, -10.0, 10.0, 12U), value, one_lsb);
  }
}

TEST(FrameCodecTest, FloatMappingRejectsInvalidInputs)
{
  EXPECT_THROW(static_cast<void>(float_to_uint(0.0, 1.0, 1.0, 12U)), std::invalid_argument);
  EXPECT_THROW(static_cast<void>(float_to_uint(0.0, -1.0, 1.0, 0U)), std::invalid_argument);
  EXPECT_THROW(static_cast<void>(float_to_uint(0.0, -1.0, 1.0, 32U)), std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(float_to_uint(
        std::numeric_limits<double>::quiet_NaN(), -1.0, 1.0, 12U)),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(uint_to_float(4096U, -1.0, 1.0, 12U)), std::invalid_argument);
}

TEST(FrameCodecTest, PacksMitCommandAtFieldBoundaries)
{
  const MitCommand command{12.5, -30.0, 500.0, 0.0, 10.0};
  const auto frame = encode_mit_command(0x123U, command, kLimits);

  EXPECT_EQ(frame.id, 0x123U);
  EXPECT_EQ(frame.length, 8U);
  const std::array<std::uint8_t, 8U> expected{
    0xFFU, 0xFFU, 0x00U, 0x0FU, 0xFFU, 0x00U, 0x0FU, 0xFFU};
  EXPECT_EQ(frame.data, expected);
}

TEST(FrameCodecTest, DecodesMitCommandWithSplitNibbles)
{
  const CanFrame frame{
    0x123U, 8U,
    {0x12U, 0x34U, 0x56U, 0x78U, 0x9AU, 0xBCU, 0xDEU, 0xF0U}};

  const auto decoded = decode_mit_command(frame, kLimits);
  EXPECT_NEAR(decoded.position, uint_to_float(0x1234U, -12.5, 12.5, 16U), 1e-12);
  EXPECT_NEAR(decoded.velocity, uint_to_float(0x567U, -30.0, 30.0, 12U), 1e-12);
  EXPECT_NEAR(decoded.kp, uint_to_float(0x89AU, 0.0, 500.0, 12U), 1e-12);
  EXPECT_NEAR(decoded.kd, uint_to_float(0xBCDU, 0.0, 5.0, 12U), 1e-12);
  EXPECT_NEAR(decoded.torque, uint_to_float(0xEF0U, -10.0, 10.0, 12U), 1e-12);
}

TEST(FrameCodecTest, DecodesFeedbackWithErrorAndSplitNibbles)
{
  const CanFrame frame{
    0x321U, 8U,
    {0xA3U, 0xFFU, 0xFFU, 0xABU, 0xC1U, 0x23U, 72U, 65U}};

  const auto feedback = decode_motor_feedback(frame, kLimits);
  EXPECT_EQ(feedback.motor_id, 3U);
  EXPECT_EQ(feedback.error, MotorError::over_current);
  EXPECT_NEAR(feedback.position, 12.5, 1e-12);
  EXPECT_NEAR(feedback.velocity, uint_to_float(0xABCU, -30.0, 30.0, 12U), 1e-12);
  EXPECT_NEAR(feedback.torque, uint_to_float(0x123U, -10.0, 10.0, 12U), 1e-12);
  EXPECT_EQ(feedback.mos_temperature_c, 72U);
  EXPECT_EQ(feedback.rotor_temperature_c, 65U);
}

TEST(FrameCodecTest, FeedbackEncodingMatchesProtocolBytes)
{
  const MotorFeedback feedback{
    0x0EU, MotorError::communication_lost, 12.5, -30.0, 10.0, 70U, 60U};
  const auto frame = encode_motor_feedback(0x456U, feedback, kLimits);

  const std::array<std::uint8_t, 8U> expected{
    0xDEU, 0xFFU, 0xFFU, 0x00U, 0x0FU, 0xFFU, 70U, 60U};
  EXPECT_EQ(frame.id, 0x456U);
  EXPECT_EQ(frame.data, expected);
}

TEST(FrameCodecTest, SpecialCommandsAreSevenFfBytesAndOpcode)
{
  const auto frame = make_special_command(7U, SpecialCommand::enable);
  EXPECT_EQ(frame.id, 7U);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8U>{
      0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU}));
  ASSERT_TRUE(decode_special_command(frame).has_value());
  EXPECT_EQ(*decode_special_command(frame), SpecialCommand::enable);

  CanFrame ordinary = frame;
  ordinary.data[0] = 0U;
  EXPECT_FALSE(decode_special_command(ordinary).has_value());
}

TEST(FrameCodecTest, RegisterFramesUseDocumentedByteOrder)
{
  const auto read = make_register_read(0x345U, 0x15U);
  EXPECT_EQ(read.id, kRegisterCanId);
  EXPECT_EQ(read.data, (std::array<std::uint8_t, 8U>{
      0x45U, 0x03U, 0x33U, 0x15U, 0U, 0U, 0U, 0U}));

  const auto write = make_register_write_float(0x345U, 0x16U, 1.0F);
  EXPECT_EQ(write.data, (std::array<std::uint8_t, 8U>{
      0x45U, 0x03U, 0x55U, 0x16U, 0x00U, 0x00U, 0x80U, 0x3FU}));
}

TEST(FrameCodecTest, DecodesRegisterReplyPayload)
{
  const CanFrame frame{
    0x22U, 8U,
    {0x45U, 0x03U, 0x33U, 0x15U, 0x00U, 0x00U, 0x48U, 0x41U}};
  const auto reply = decode_register_reply(frame);
  EXPECT_EQ(reply.motor_can_id, 0x345U);
  EXPECT_EQ(reply.operation, RegisterOperation::read);
  EXPECT_EQ(reply.register_id, 0x15U);
  EXPECT_EQ(reply.raw_value, 0x41480000U);
  EXPECT_FLOAT_EQ(reply.float_value(), 12.5F);
}

TEST(FrameCodecTest, RejectsMalformedFramesAndIdentifiers)
{
  CanFrame short_frame{};
  short_frame.length = 7U;
  EXPECT_THROW(
    static_cast<void>(decode_motor_feedback(short_frame, kLimits)), std::invalid_argument);
  EXPECT_THROW(static_cast<void>(decode_register_reply(short_frame)), std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(make_special_command(0x800U, SpecialCommand::enable)),
    std::invalid_argument);

  CanFrame bad_register{};
  bad_register.data[2] = 0x44U;
  EXPECT_THROW(static_cast<void>(decode_register_reply(bad_register)), std::invalid_argument);
}

}  // namespace
}  // namespace dm_swerve_driver
