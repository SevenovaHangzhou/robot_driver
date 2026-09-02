#include "dm_swerve_driver/dm_frame_codec.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace dm_swerve_driver {
namespace {

constexpr unsigned int kMaximumMappingBits{31U};
constexpr std::uint16_t kMaximumStandardCanId{0x7FFU};

[[nodiscard]] std::uint32_t mapping_maximum(unsigned int bits)
{
  if (bits == 0U || bits > kMaximumMappingBits) {
    throw std::invalid_argument{"mapping bit width must be in [1, 31]"};
  }
  return (std::uint32_t{1U} << bits) - 1U;
}

void validate_mapping_range(double minimum, double maximum)
{
  if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum >= maximum) {
    throw std::invalid_argument{"mapping range must be finite and increasing"};
  }
}

void validate_standard_id(std::uint16_t can_id)
{
  if (can_id > kMaximumStandardCanId) {
    throw std::invalid_argument{"CAN identifier exceeds the 11-bit standard-frame range"};
  }
}

void validate_protocol_frame(const CanFrame & frame)
{
  validate_standard_id(frame.id);
  if (frame.length != kCanPayloadSize) {
    throw std::invalid_argument{"DaMiao protocol frames must contain eight bytes"};
  }
}

void validate_limits(const MotorLimits & limits)
{
  if (!limits.valid()) {
    throw std::invalid_argument{"motor mapping limits must be finite and positive"};
  }
}

[[nodiscard]] std::uint16_t join_u16(std::uint8_t high, std::uint8_t low) noexcept
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(high) << 8U) | static_cast<std::uint16_t>(low));
}

[[nodiscard]] std::uint32_t read_little_endian_u32(
  const std::array<std::uint8_t, kCanPayloadSize> & data) noexcept
{
  return static_cast<std::uint32_t>(data[4]) |
         (static_cast<std::uint32_t>(data[5]) << 8U) |
         (static_cast<std::uint32_t>(data[6]) << 16U) |
         (static_cast<std::uint32_t>(data[7]) << 24U);
}

void write_little_endian_u32(
  std::array<std::uint8_t, kCanPayloadSize> & data, std::uint32_t value) noexcept
{
  data[4] = static_cast<std::uint8_t>(value & 0xFFU);
  data[5] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  data[6] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  data[7] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

[[nodiscard]] CanFrame make_register_frame(
  std::uint16_t motor_can_id, std::uint8_t register_id,
  RegisterOperation operation, std::uint32_t raw_value)
{
  validate_standard_id(motor_can_id);
  CanFrame frame{};
  frame.id = kRegisterCanId;
  frame.data[0] = static_cast<std::uint8_t>(motor_can_id & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>((motor_can_id >> 8U) & 0xFFU);
  frame.data[2] = static_cast<std::uint8_t>(operation);
  frame.data[3] = register_id;
  write_little_endian_u32(frame.data, raw_value);
  return frame;
}

}  // namespace

bool MotorLimits::valid() const noexcept
{
  return std::isfinite(position_max) && position_max > 0.0 &&
         std::isfinite(velocity_max) && velocity_max > 0.0 &&
         std::isfinite(torque_max) && torque_max > 0.0;
}

float RegisterReply::float_value() const noexcept
{
  static_assert(sizeof(float) == sizeof(raw_value), "protocol requires a 32-bit float");
  float value{0.0F};
  std::memcpy(&value, &raw_value, sizeof(value));
  return value;
}

std::uint32_t float_to_uint(
  double value, double minimum, double maximum, unsigned int bits)
{
  validate_mapping_range(minimum, maximum);
  const std::uint32_t encoded_maximum{mapping_maximum(bits)};
  if (!std::isfinite(value)) {
    throw std::invalid_argument{"mapped value must be finite"};
  }

  const double clamped{std::clamp(value, minimum, maximum)};
  const double scaled{
    (clamped - minimum) * static_cast<double>(encoded_maximum) / (maximum - minimum)};
  return static_cast<std::uint32_t>(scaled);
}

double uint_to_float(
  std::uint32_t value, double minimum, double maximum, unsigned int bits)
{
  validate_mapping_range(minimum, maximum);
  const std::uint32_t encoded_maximum{mapping_maximum(bits)};
  if (value > encoded_maximum) {
    throw std::invalid_argument{"encoded value exceeds its declared bit width"};
  }
  return static_cast<double>(value) * (maximum - minimum) /
         static_cast<double>(encoded_maximum) + minimum;
}

CanFrame encode_mit_command(
  std::uint16_t esc_id, const MitCommand & command, const MotorLimits & limits)
{
  validate_standard_id(esc_id);
  validate_limits(limits);

  const std::uint32_t position{
    float_to_uint(command.position, -limits.position_max, limits.position_max, 16U)};
  const std::uint32_t velocity{
    float_to_uint(command.velocity, -limits.velocity_max, limits.velocity_max, 12U)};
  const std::uint32_t kp{float_to_uint(command.kp, 0.0, kMitKpMax, 12U)};
  const std::uint32_t kd{float_to_uint(command.kd, 0.0, kMitKdMax, 12U)};
  const std::uint32_t torque{
    float_to_uint(command.torque, -limits.torque_max, limits.torque_max, 12U)};

  CanFrame frame{};
  frame.id = esc_id;
  frame.data[0] = static_cast<std::uint8_t>((position >> 8U) & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>(position & 0xFFU);
  frame.data[2] = static_cast<std::uint8_t>((velocity >> 4U) & 0xFFU);
  frame.data[3] = static_cast<std::uint8_t>(((velocity & 0x0FU) << 4U) | (kp >> 8U));
  frame.data[4] = static_cast<std::uint8_t>(kp & 0xFFU);
  frame.data[5] = static_cast<std::uint8_t>((kd >> 4U) & 0xFFU);
  frame.data[6] = static_cast<std::uint8_t>(((kd & 0x0FU) << 4U) | (torque >> 8U));
  frame.data[7] = static_cast<std::uint8_t>(torque & 0xFFU);
  return frame;
}

MitCommand decode_mit_command(const CanFrame & frame, const MotorLimits & limits)
{
  validate_protocol_frame(frame);
  validate_limits(limits);

  const std::uint32_t position{join_u16(frame.data[0], frame.data[1])};
  const std::uint32_t velocity{
    (static_cast<std::uint32_t>(frame.data[2]) << 4U) |
    (static_cast<std::uint32_t>(frame.data[3]) >> 4U)};
  const std::uint32_t kp{
    ((static_cast<std::uint32_t>(frame.data[3]) & 0x0FU) << 8U) | frame.data[4]};
  const std::uint32_t kd{
    (static_cast<std::uint32_t>(frame.data[5]) << 4U) |
    (static_cast<std::uint32_t>(frame.data[6]) >> 4U)};
  const std::uint32_t torque{
    ((static_cast<std::uint32_t>(frame.data[6]) & 0x0FU) << 8U) | frame.data[7]};

  return MitCommand{
    uint_to_float(position, -limits.position_max, limits.position_max, 16U),
    uint_to_float(velocity, -limits.velocity_max, limits.velocity_max, 12U),
    uint_to_float(kp, 0.0, kMitKpMax, 12U),
    uint_to_float(kd, 0.0, kMitKdMax, 12U),
    uint_to_float(torque, -limits.torque_max, limits.torque_max, 12U)};
}

CanFrame encode_motor_feedback(
  std::uint16_t mst_id, const MotorFeedback & feedback, const MotorLimits & limits)
{
  validate_standard_id(mst_id);
  validate_limits(limits);
  if (feedback.motor_id > 0x0FU) {
    throw std::invalid_argument{"feedback motor identifier must fit in four bits"};
  }

  const std::uint32_t position{
    float_to_uint(feedback.position, -limits.position_max, limits.position_max, 16U)};
  const std::uint32_t velocity{
    float_to_uint(feedback.velocity, -limits.velocity_max, limits.velocity_max, 12U)};
  const std::uint32_t torque{
    float_to_uint(feedback.torque, -limits.torque_max, limits.torque_max, 12U)};

  CanFrame frame{};
  frame.id = mst_id;
  frame.data[0] = static_cast<std::uint8_t>(
    (static_cast<std::uint8_t>(feedback.error) << 4U) | feedback.motor_id);
  frame.data[1] = static_cast<std::uint8_t>((position >> 8U) & 0xFFU);
  frame.data[2] = static_cast<std::uint8_t>(position & 0xFFU);
  frame.data[3] = static_cast<std::uint8_t>((velocity >> 4U) & 0xFFU);
  frame.data[4] = static_cast<std::uint8_t>(((velocity & 0x0FU) << 4U) | (torque >> 8U));
  frame.data[5] = static_cast<std::uint8_t>(torque & 0xFFU);
  frame.data[6] = feedback.mos_temperature_c;
  frame.data[7] = feedback.rotor_temperature_c;
  return frame;
}

MotorFeedback decode_motor_feedback(const CanFrame & frame, const MotorLimits & limits)
{
  validate_protocol_frame(frame);
  validate_limits(limits);

  const std::uint32_t position{join_u16(frame.data[1], frame.data[2])};
  const std::uint32_t velocity{
    (static_cast<std::uint32_t>(frame.data[3]) << 4U) |
    (static_cast<std::uint32_t>(frame.data[4]) >> 4U)};
  const std::uint32_t torque{
    ((static_cast<std::uint32_t>(frame.data[4]) & 0x0FU) << 8U) | frame.data[5]};

  return MotorFeedback{
    static_cast<std::uint8_t>(frame.data[0] & 0x0FU),
    static_cast<MotorError>(frame.data[0] >> 4U),
    uint_to_float(position, -limits.position_max, limits.position_max, 16U),
    uint_to_float(velocity, -limits.velocity_max, limits.velocity_max, 12U),
    uint_to_float(torque, -limits.torque_max, limits.torque_max, 12U),
    frame.data[6],
    frame.data[7]};
}

CanFrame make_special_command(std::uint16_t esc_id, SpecialCommand command)
{
  validate_standard_id(esc_id);
  CanFrame frame{};
  frame.id = esc_id;
  frame.data.fill(0xFFU);
  frame.data.back() = static_cast<std::uint8_t>(command);
  return frame;
}

std::optional<SpecialCommand> decode_special_command(const CanFrame & frame) noexcept
{
  if (frame.id > kMaximumStandardCanId || frame.length != kCanPayloadSize ||
    !std::all_of(frame.data.begin(), frame.data.end() - 1, [](std::uint8_t value) {
      return value == 0xFFU;
    }))
  {
    return std::nullopt;
  }

  switch (frame.data.back()) {
    case static_cast<std::uint8_t>(SpecialCommand::clear_fault):
      return SpecialCommand::clear_fault;
    case static_cast<std::uint8_t>(SpecialCommand::enable):
      return SpecialCommand::enable;
    case static_cast<std::uint8_t>(SpecialCommand::disable):
      return SpecialCommand::disable;
    case static_cast<std::uint8_t>(SpecialCommand::save_zero):
      return SpecialCommand::save_zero;
    default:
      return std::nullopt;
  }
}

CanFrame make_register_read(std::uint16_t motor_can_id, std::uint8_t register_id)
{
  return make_register_frame(motor_can_id, register_id, RegisterOperation::read, 0U);
}

CanFrame make_register_write(
  std::uint16_t motor_can_id, std::uint8_t register_id, std::uint32_t raw_value)
{
  return make_register_frame(motor_can_id, register_id, RegisterOperation::write, raw_value);
}

CanFrame make_register_write_float(
  std::uint16_t motor_can_id, std::uint8_t register_id, float value)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t), "protocol requires a 32-bit float");
  if (!std::isfinite(value)) {
    throw std::invalid_argument{"register float must be finite"};
  }
  std::uint32_t raw_value{0U};
  std::memcpy(&raw_value, &value, sizeof(raw_value));
  return make_register_write(motor_can_id, register_id, raw_value);
}

RegisterReply decode_register_reply(const CanFrame & frame)
{
  validate_protocol_frame(frame);
  const auto operation_byte = frame.data[2];
  if (operation_byte != static_cast<std::uint8_t>(RegisterOperation::read) &&
    operation_byte != static_cast<std::uint8_t>(RegisterOperation::write))
  {
    throw std::invalid_argument{"unknown register reply operation"};
  }

  const auto motor_can_id = static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(frame.data[0]) |
    (static_cast<std::uint16_t>(frame.data[1]) << 8U));
  validate_standard_id(motor_can_id);
  return RegisterReply{
    motor_can_id,
    static_cast<RegisterOperation>(operation_byte),
    frame.data[3],
    read_little_endian_u32(frame.data)};
}

}  // namespace dm_swerve_driver
