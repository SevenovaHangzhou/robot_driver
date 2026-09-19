#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "robot_hw_can/protocol.hpp"

namespace robot_hw_can
{
namespace
{

TEST(DamiaoProtocolTest, EncodesPositionVelocityAndSpecialFrames)
{
  const auto frame = make_position_velocity_command(2U, 1.0F, 0.5F);
  EXPECT_EQ(frame.id, 0x102U);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8U>{
    0x00U, 0x00U, 0x80U, 0x3FU, 0x00U, 0x00U, 0x00U, 0x3FU}));
  const auto enable = make_special_command(1U, SpecialCommand::enable);
  EXPECT_EQ(enable.id, 0x101U);
  EXPECT_EQ(enable.data, (std::array<std::uint8_t, 8U>{
    0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU}));
  EXPECT_EQ(make_special_command(2U, SpecialCommand::disable).data[7], 0xFDU);
  EXPECT_THROW(static_cast<void>(make_position_velocity_command(0U, 0.0F, 1.0F)),
    std::invalid_argument);
  EXPECT_THROW(static_cast<void>(make_position_velocity_command(2U, NAN, 1.0F)),
    std::invalid_argument);
  EXPECT_THROW(static_cast<void>(make_position_velocity_command(2U, 0.0F, 0.0F)),
    std::invalid_argument);
}

TEST(DamiaoProtocolTest, RegisterReadAndReplyUseLittleEndian)
{
  const auto read = make_register_read(2U, Register::control_mode);
  EXPECT_EQ(read.id, 0x7FFU);
  EXPECT_EQ(read.data, (std::array<std::uint8_t, 8U>{
    0x02U, 0x00U, 0x33U, 0x0AU, 0U, 0U, 0U, 0U}));

  CanFrame reply{};
  reply.id = 0x12U;
  reply.data = {0x02U, 0U, 0x33U, 0x15U, 0U, 0U, 0x48U, 0x41U};
  const auto decoded = decode_register_reply(reply);
  EXPECT_EQ(decoded.motor_id, 2U);
  EXPECT_EQ(decoded.register_id, 0x15U);
  EXPECT_FLOAT_EQ(decoded.float_value(), 12.5F);
  reply.data[2] = 0x99U;
  EXPECT_THROW(static_cast<void>(decode_register_reply(reply)), std::invalid_argument);
}

TEST(DamiaoProtocolTest, ValidatesReadOnlyIdentityBitrateAndWatchdogRegisters)
{
  EXPECT_EQ(make_register_read(1U, Register::can_id).data[3], 0x08U);
  EXPECT_EQ(make_register_read(1U, Register::master_id).data[3], 0x07U);
  EXPECT_EQ(make_register_read(1U, Register::can_timeout).data[3], 0x09U);
  EXPECT_EQ(make_register_read(1U, Register::can_bitrate).data[3], 0x23U);

  MotorConfiguration registers{1U, 0x11U, 2000U, 4U};
  EXPECT_NO_THROW(validate_motor_configuration(registers, 1U, 0x11U));
  EXPECT_THROW(validate_motor_configuration(registers, 2U, 0x11U), std::invalid_argument);
  EXPECT_THROW(validate_motor_configuration(registers, 1U, 0x12U), std::invalid_argument);
  registers.bitrate_code = 3U;
  EXPECT_THROW(validate_motor_configuration(registers, 1U, 0x11U), std::invalid_argument);
  registers.bitrate_code = 4U;
  registers.timeout_ticks = 0U;
  EXPECT_THROW(validate_motor_configuration(registers, 1U, 0x11U), std::invalid_argument);
}

TEST(DamiaoProtocolTest, DecodesFeedbackUsingReadbackMapping)
{
  CanFrame feedback{};
  feedback.id = 0x11U;
  feedback.data = {0x11U, 0xFFU, 0xFFU, 0x80U, 0x08U, 0x00U, 35U, 36U};
  const auto value = decode_feedback(feedback, MotorLimits{12.5, 30.0, 10.0});
  EXPECT_EQ(value.motor_id, 1U);
  EXPECT_EQ(value.status, 1U);
  EXPECT_DOUBLE_EQ(value.position, 12.5);
  EXPECT_NEAR(value.velocity, 0.0, 0.02);
  EXPECT_NEAR(value.effort, 0.0, 0.01);
  EXPECT_DOUBLE_EQ(value.mos_temperature_c, 35.0);
  feedback.data[0] = 0x91U;
  EXPECT_EQ(decode_feedback(feedback, MotorLimits{12.5, 30.0, 10.0}).status, 9U);
  EXPECT_THROW(static_cast<void>(decode_feedback(feedback, MotorLimits{})),
    std::invalid_argument);
}

}  // namespace
}  // namespace robot_hw_can
