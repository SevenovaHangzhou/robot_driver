#include "dm_swerve_driver/dm_motor.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace dm_swerve_driver {
namespace {

constexpr std::uint16_t kMaximumStandardCanId{0x7FFU};
void validate_limits(const MotorLimits & limits)
{
  if (!limits.valid()) {
    throw std::invalid_argument{"DmMotor limits must be finite and positive"};
  }
}

void validate_raw_position(double raw_position, const MotorLimits & limits)
{
  if (!std::isfinite(raw_position) || std::abs(raw_position) > limits.position_max) {
    throw std::invalid_argument{"raw motor position is outside PMAX"};
  }
}

[[nodiscard]] double wrap_pi(double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void increment_saturated(std::uint64_t & counter) noexcept
{
  if (counter != std::numeric_limits<std::uint64_t>::max()) {
    ++counter;
  }
}

}  // namespace

bool DmMotorHealth::enabled() const noexcept
{
  return has_feedback && error == MotorError::enabled;
}

bool DmMotorHealth::has_fault() const noexcept
{
  return has_feedback && error != MotorError::disabled && error != MotorError::enabled;
}

DmMotor::DmMotor(DmMotorConfig config)
: config_{std::move(config)}
{
  if (config_.esc_id > kMaximumStandardCanId || config_.mst_id > kMaximumStandardCanId) {
    throw std::invalid_argument{"DmMotor CAN identifiers must fit in 11 bits"};
  }
  validate_limits(config_.limits);
}

std::uint16_t DmMotor::esc_id() const noexcept
{
  return config_.esc_id;
}

std::uint16_t DmMotor::mst_id() const noexcept
{
  return config_.mst_id;
}

const MotorLimits & DmMotor::limits() const noexcept
{
  return config_.limits;
}

void DmMotor::set_limits(const MotorLimits & limits)
{
  validate_limits(limits);
  if (position_initialized_) {
    throw std::logic_error{"motor limits cannot change after position initialization"};
  }
  config_.limits = limits;
}

void DmMotor::seed_position(double raw_position)
{
  validate_raw_position(raw_position, config_.limits);
  health_.seeded_from_multi_turn = false;
  position_initialized_ = true;
  raw_position_ = raw_position;
  position_offset_ = 0.0;
  unwrapped_position_ = raw_position;
}

bool DmMotor::seed_position_from_multi_turn(
  double raw_position, double multi_turn_position, double consistency_tolerance_rad)
{
  validate_raw_position(raw_position, config_.limits);
  if (!std::isfinite(multi_turn_position) || !std::isfinite(consistency_tolerance_rad) ||
    consistency_tolerance_rad < 0.0)
  {
    throw std::invalid_argument{"multi-turn seed and tolerance must be finite and valid"};
  }

  health_.seeded_from_multi_turn = true;
  position_initialized_ = true;
  raw_position_ = raw_position;
  position_offset_ = multi_turn_position - raw_position;
  unwrapped_position_ = multi_turn_position;
  return multi_turn_consistent(raw_position, multi_turn_position, consistency_tolerance_rad);
}

bool DmMotor::multi_turn_consistent(
  double raw_position, double multi_turn_position, double tolerance_rad)
{
  if (!std::isfinite(raw_position) || !std::isfinite(multi_turn_position) ||
    !std::isfinite(tolerance_rad) || tolerance_rad < 0.0)
  {
    throw std::invalid_argument{"multi-turn consistency inputs must be finite and valid"};
  }
  const double difference{wrap_pi(wrap_pi(multi_turn_position) - wrap_pi(raw_position))};
  return std::abs(difference) < tolerance_rad;
}

MotorFeedback DmMotor::accept_feedback(const CanFrame & frame)
{
  if (frame.id != config_.mst_id) {
    throw std::invalid_argument{"feedback CAN identifier does not match motor MST_ID"};
  }
  const MotorFeedback feedback{decode_motor_feedback(frame, config_.limits)};
  if (feedback.motor_id != static_cast<std::uint8_t>(config_.esc_id & 0x0FU)) {
    throw std::invalid_argument{"feedback payload motor identifier does not match ESC_ID"};
  }

  if (position_initialized_) {
    update_position(feedback.position);
  } else {
    seed_position(feedback.position);
  }
  velocity_ = feedback.velocity;
  torque_ = feedback.torque;
  health_.has_feedback = true;
  health_.error = feedback.error;
  health_.mos_temperature_c = feedback.mos_temperature_c;
  health_.rotor_temperature_c = feedback.rotor_temperature_c;
  increment_saturated(health_.received_frames);
  health_.consecutive_missed_frames = 0U;
  return feedback;
}

void DmMotor::mark_feedback_missed() noexcept
{
  increment_saturated(health_.missed_frames);
  increment_saturated(health_.consecutive_missed_frames);
}

bool DmMotor::position_initialized() const noexcept
{
  return position_initialized_;
}

double DmMotor::raw_position() const noexcept
{
  return raw_position_;
}

double DmMotor::unwrapped_position() const noexcept
{
  return unwrapped_position_;
}

double DmMotor::velocity() const noexcept
{
  return velocity_;
}

double DmMotor::torque() const noexcept
{
  return torque_;
}

const DmMotorHealth & DmMotor::health() const noexcept
{
  return health_;
}

CanFrame DmMotor::encode_command(const MitCommand & command) const
{
  return encode_mit_command(config_.esc_id, command, config_.limits);
}

CanFrame DmMotor::special_command(SpecialCommand command) const
{
  return make_special_command(config_.esc_id, command);
}

void DmMotor::update_position(double raw_position) noexcept
{
  const double difference{raw_position - raw_position_};
  if (difference > config_.limits.position_max) {
    position_offset_ -= 2.0 * config_.limits.position_max;
  } else if (difference < -config_.limits.position_max) {
    position_offset_ += 2.0 * config_.limits.position_max;
  }
  raw_position_ = raw_position;
  unwrapped_position_ = raw_position + position_offset_;
}

}  // namespace dm_swerve_driver
