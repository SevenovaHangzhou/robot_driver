#include "robot_hw_can/protocol.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace robot_hw_can
{
namespace
{

constexpr std::uint16_t kStandardCanIdMaximum{0x7FFU};
constexpr std::uint16_t kPositionVelocityIdOffset{0x100U};

void validate_motor_id(std::uint16_t motor_id)
{
  if (motor_id == 0U || motor_id > 0x0FU) {
    throw std::invalid_argument{"DaMiao motor ID must be in [1, 15]"};
  }
}

void validate_frame(const CanFrame & frame)
{
  if (frame.id > kStandardCanIdMaximum || frame.length != kPayloadSize) {
    throw std::invalid_argument{"DaMiao response must be an eight-byte standard CAN frame"};
  }
}

void write_float_le(
  std::array<std::uint8_t, kPayloadSize> & data, std::size_t offset, float value)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t), "DaMiao protocol requires float32");
  if (!std::isfinite(value)) {
    throw std::invalid_argument{"DaMiao float command must be finite"};
  }
  std::uint32_t raw{0U};
  std::memcpy(&raw, &value, sizeof(raw));
  for (std::size_t byte{0U}; byte < sizeof(raw); ++byte) {
    data[offset + byte] = static_cast<std::uint8_t>((raw >> (8U * byte)) & 0xFFU);
  }
}

[[nodiscard]] std::uint32_t read_u32_le(
  const std::array<std::uint8_t, kPayloadSize> & data, std::size_t offset) noexcept
{
  return static_cast<std::uint32_t>(data[offset]) |
         (static_cast<std::uint32_t>(data[offset + 1U]) << 8U) |
         (static_cast<std::uint32_t>(data[offset + 2U]) << 16U) |
         (static_cast<std::uint32_t>(data[offset + 3U]) << 24U);
}

[[nodiscard]] double map_unsigned(
  std::uint32_t raw, double maximum, std::uint32_t encoded_maximum)
{
  return static_cast<double>(raw) * (2.0 * maximum) /
         static_cast<double>(encoded_maximum) - maximum;
}

}  // namespace

bool MotorLimits::valid() const noexcept
{
  return std::isfinite(position_max) && position_max > 0.0 &&
         std::isfinite(velocity_max) && velocity_max > 0.0 &&
         std::isfinite(torque_max) && torque_max > 0.0;
}

bool TrapezoidalProfile::valid() const noexcept
{
  return std::isfinite(acceleration_krad_s2) && acceleration_krad_s2 > 0.0 &&
         std::isfinite(deceleration_krad_s2) && deceleration_krad_s2 < 0.0 &&
         std::isfinite(maximum_speed_rad_s) && maximum_speed_rad_s > 0.0;
}

float RegisterReply::float_value() const noexcept
{
  float value{0.0F};
  std::memcpy(&value, &raw_value, sizeof(value));
  return value;
}

CanFrame make_position_velocity_command(
  std::uint16_t motor_id, float position_rad, float velocity_limit_rad_s)
{
  validate_motor_id(motor_id);
  if (velocity_limit_rad_s <= 0.0F) {
    throw std::invalid_argument{"DaMiao velocity limit must be positive"};
  }
  CanFrame frame{};
  frame.id = static_cast<std::uint16_t>(kPositionVelocityIdOffset + motor_id);
  write_float_le(frame.data, 0U, position_rad);
  write_float_le(frame.data, 4U, velocity_limit_rad_s);
  return frame;
}

CanFrame make_special_command(std::uint16_t motor_id, SpecialCommand command)
{
  validate_motor_id(motor_id);
  CanFrame frame{};
  frame.id = static_cast<std::uint16_t>(kPositionVelocityIdOffset + motor_id);
  frame.data.fill(0xFFU);
  frame.data.back() = static_cast<std::uint8_t>(command);
  return frame;
}

CanFrame make_register_read(std::uint16_t motor_id, Register register_id)
{
  validate_motor_id(motor_id);
  CanFrame frame{};
  frame.id = kRegisterRequestId;
  frame.data[0] = static_cast<std::uint8_t>(motor_id & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>((motor_id >> 8U) & 0xFFU);
  frame.data[2] = 0x33U;
  frame.data[3] = static_cast<std::uint8_t>(register_id);
  return frame;
}

CanFrame make_register_write_float(
  std::uint16_t motor_id, Register register_id, float value)
{
  validate_motor_id(motor_id);
  CanFrame frame{};
  frame.id = kRegisterRequestId;
  frame.data[0] = static_cast<std::uint8_t>(motor_id & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>((motor_id >> 8U) & 0xFFU);
  frame.data[2] = 0x55U;
  frame.data[3] = static_cast<std::uint8_t>(register_id);
  write_float_le(frame.data, 4U, value);
  return frame;
}

CanFrame make_parameter_save(std::uint16_t motor_id)
{
  validate_motor_id(motor_id);
  CanFrame frame{};
  frame.id = kRegisterRequestId;
  frame.data[0] = static_cast<std::uint8_t>(motor_id & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>((motor_id >> 8U) & 0xFFU);
  frame.data[2] = 0xAAU;
  frame.data[3] = 0x01U;
  return frame;
}

RegisterReply decode_register_reply(const CanFrame & frame)
{
  validate_frame(frame);
  if (frame.data[2] != 0x33U && frame.data[2] != 0x55U) {
    throw std::invalid_argument{"DaMiao register response has an unknown operation"};
  }
  const auto motor_id = static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(frame.data[0]) |
    (static_cast<std::uint16_t>(frame.data[1]) << 8U));
  if (motor_id > kStandardCanIdMaximum) {
    throw std::invalid_argument{"DaMiao register response contains an invalid motor ID"};
  }
  return RegisterReply{motor_id, frame.data[2], frame.data[3], read_u32_le(frame.data, 4U)};
}

void validate_motor_configuration(
  const MotorConfiguration & configuration, std::uint16_t expected_can_id,
  std::uint16_t expected_master_id)
{
  if (configuration.can_id != expected_can_id) {
    throw std::invalid_argument{"DaMiao CAN ID does not match the configured joint"};
  }
  if (configuration.master_id != expected_master_id) {
    throw std::invalid_argument{"DaMiao Master ID does not match the configured joint"};
  }
  if (configuration.bitrate_code != kOneMegabitCanCode) {
    throw std::invalid_argument{"DaMiao motor must already be configured for 1 Mbit/s"};
  }
  if (configuration.timeout_ticks == 0U) {
    throw std::invalid_argument{"DaMiao motor CAN timeout protection must be enabled"};
  }
}

void validate_trapezoidal_profile(
  const TrapezoidalProfile & actual, const TrapezoidalProfile & expected)
{
  if (!actual.valid() || !expected.valid()) {
    throw std::invalid_argument{"DaMiao trapezoidal profile values have invalid signs or ranges"};
  }
  const auto close = [](double lhs, double rhs) {
      constexpr double kAbsoluteTolerance{1.0e-6};
      constexpr double kRelativeTolerance{1.0e-4};
      return std::abs(lhs - rhs) <=
             std::max(kAbsoluteTolerance, std::abs(rhs) * kRelativeTolerance);
    };
  if (!close(actual.acceleration_krad_s2, expected.acceleration_krad_s2) ||
    !close(actual.deceleration_krad_s2, expected.deceleration_krad_s2) ||
    !close(actual.maximum_speed_rad_s, expected.maximum_speed_rad_s))
  {
    throw std::invalid_argument{"DaMiao stored trapezoidal profile does not match configuration"};
  }
}

MotorFeedback decode_feedback(const CanFrame & frame, const MotorLimits & limits)
{
  validate_frame(frame);
  if (!limits.valid()) {
    throw std::invalid_argument{"DaMiao feedback mapping limits must be finite and positive"};
  }
  const std::uint32_t position =
    (static_cast<std::uint32_t>(frame.data[1]) << 8U) | frame.data[2];
  const std::uint32_t velocity =
    (static_cast<std::uint32_t>(frame.data[3]) << 4U) | (frame.data[4] >> 4U);
  const std::uint32_t effort =
    (static_cast<std::uint32_t>(frame.data[4] & 0x0FU) << 8U) | frame.data[5];
  return MotorFeedback{
    static_cast<std::uint8_t>(frame.data[0] & 0x0FU),
    static_cast<std::uint8_t>(frame.data[0] >> 4U),
    map_unsigned(position, limits.position_max, 0xFFFFU),
    map_unsigned(velocity, limits.velocity_max, 0x0FFFU),
    map_unsigned(effort, limits.torque_max, 0x0FFFU),
    static_cast<double>(frame.data[6]),
    static_cast<double>(frame.data[7])};
}

}  // namespace robot_hw_can
