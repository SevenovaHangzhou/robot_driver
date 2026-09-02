#include "dm_swerve_driver/fake_motor_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace dm_swerve_driver {
namespace {

[[nodiscard]] std::uint32_t float_bits(float value) noexcept
{
  static_assert(sizeof(float) == sizeof(std::uint32_t), "protocol requires a 32-bit float");
  std::uint32_t bits{0U};
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

[[nodiscard]] std::uint32_t register_value(
  std::uint8_t register_id, const FakeMotorConfig & config,
  double multi_turn_position, std::uint32_t timeout_register) noexcept
{
  if (register_id == static_cast<std::uint8_t>(RegisterId::position_max)) {
    return float_bits(static_cast<float>(config.limits.position_max));
  }
  if (register_id == static_cast<std::uint8_t>(RegisterId::velocity_max)) {
    return float_bits(static_cast<float>(config.limits.velocity_max));
  }
  if (register_id == static_cast<std::uint8_t>(RegisterId::torque_max)) {
    return float_bits(static_cast<float>(config.limits.torque_max));
  }
  if (register_id == static_cast<std::uint8_t>(RegisterId::multi_turn_position)) {
    return float_bits(static_cast<float>(multi_turn_position));
  }
  if (register_id == static_cast<std::uint8_t>(RegisterId::timeout)) {
    return timeout_register;
  }
  return 0U;
}

[[nodiscard]] double wrapped_feedback_position(double position, double position_max) noexcept
{
  const double period{2.0 * position_max};
  double wrapped{std::fmod(position + position_max, period)};
  if (wrapped < 0.0) {
    wrapped += period;
  }
  return wrapped - position_max;
}

}  // namespace

FakeMotorModel::FakeMotorModel(FakeMotorConfig config)
: config_{std::move(config)}
{
  if (config_.esc_id > 0x7FFU || config_.mst_id > 0x7FFU) {
    throw std::invalid_argument{"fake motor CAN identifiers must fit in 11 bits"};
  }
  if (!config_.limits.valid()) {
    throw std::invalid_argument{"fake motor limits must be finite and positive"};
  }
  if (!std::isfinite(config_.velocity_time_constant.count()) ||
    config_.velocity_time_constant <= std::chrono::duration<double>::zero())
  {
    throw std::invalid_argument{"fake motor time constant must be finite and positive"};
  }
}

std::uint16_t FakeMotorModel::esc_id() const noexcept
{
  return config_.esc_id;
}

std::uint16_t FakeMotorModel::mst_id() const noexcept
{
  return config_.mst_id;
}

bool FakeMotorModel::enabled() const noexcept
{
  return enabled_;
}

double FakeMotorModel::position() const noexcept
{
  return position_;
}

double FakeMotorModel::velocity() const noexcept
{
  return velocity_;
}

std::optional<CanFrame> FakeMotorModel::handle_frame(
  const CanFrame & request, std::chrono::duration<double> elapsed)
{
  if (request.id == kRegisterCanId) {
    return handle_register(request);
  }
  if (request.id != config_.esc_id) {
    return std::nullopt;
  }

  if (const auto special = decode_special_command(request); special.has_value()) {
    switch (*special) {
      case SpecialCommand::enable:
        enabled_ = true;
        break;
      case SpecialCommand::disable:
        enabled_ = false;
        velocity_ = 0.0;
        torque_ = 0.0;
        break;
      case SpecialCommand::save_zero:
        position_ = 0.0;
        break;
      case SpecialCommand::clear_fault:
        break;
    }
    return feedback_frame();
  }

  const MitCommand command{decode_mit_command(request, config_.limits)};
  if (enabled_) {
    const double elapsed_seconds{std::max(0.0, elapsed.count())};
    const double alpha{1.0 - std::exp(
        -elapsed_seconds / config_.velocity_time_constant.count())};
    const double position_velocity{command.kp * (command.position - position_) * 0.02};
    const double target_velocity{std::clamp(
        command.velocity + position_velocity,
        -config_.limits.velocity_max, config_.limits.velocity_max)};
    velocity_ += alpha * (target_velocity - velocity_);
    position_ += velocity_ * elapsed_seconds;
    torque_ = std::clamp(
      command.torque + command.kd * (command.velocity - velocity_),
      -config_.limits.torque_max, config_.limits.torque_max);
  }
  return feedback_frame();
}

CanFrame FakeMotorModel::feedback_frame() const
{
  return encode_motor_feedback(
    config_.mst_id,
    MotorFeedback{
      static_cast<std::uint8_t>(config_.esc_id & 0x0FU),
      enabled_ ? MotorError::enabled : MotorError::disabled,
      wrapped_feedback_position(position_, config_.limits.position_max),
      velocity_, torque_, 35U, 32U},
    config_.limits);
}

std::optional<CanFrame> FakeMotorModel::handle_register(const CanFrame & request)
{
  const RegisterReply request_fields{decode_register_reply(request)};
  if (request_fields.motor_can_id != config_.esc_id) {
    return std::nullopt;
  }

  std::uint32_t response_value{request_fields.raw_value};
  if (request_fields.operation == RegisterOperation::read) {
    response_value = register_value(
      request_fields.register_id, config_, position_, timeout_register_);
  } else if (request_fields.register_id == static_cast<std::uint8_t>(RegisterId::timeout)) {
    timeout_register_ = request_fields.raw_value;
  }

  CanFrame reply = request;
  reply.id = config_.mst_id;
  reply.data[4] = static_cast<std::uint8_t>(response_value & 0xFFU);
  reply.data[5] = static_cast<std::uint8_t>((response_value >> 8U) & 0xFFU);
  reply.data[6] = static_cast<std::uint8_t>((response_value >> 16U) & 0xFFU);
  reply.data[7] = static_cast<std::uint8_t>((response_value >> 24U) & 0xFFU);
  return reply;
}

}  // namespace dm_swerve_driver
