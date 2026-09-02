#include "motor_startup.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <sstream>
#include <iterator>
#include <vector>

namespace dm_swerve_driver {
namespace {

constexpr int kRegisterAttempts{3};
constexpr double kLimitComparisonTolerance{1e-4};

void log_safely(
  const StartupLogger & log, DriverLogLevel level, const std::string & message) noexcept
{
  try {
    if (log) {
      log(level, message);
    }
  } catch (...) {
  }
}

[[nodiscard]] std::chrono::steady_clock::time_point feedback_deadline(
  const DriverParameters & parameters)
{
  return std::chrono::steady_clock::now() +
         std::chrono::microseconds{parameters.can.feedback_deadline_us};
}

[[nodiscard]] std::optional<RegisterReply> exchange_register(
  CanTransport & transport,
  const DriverParameters & parameters,
  const DmMotor & motor,
  const CanFrame & request,
  std::uint8_t register_id,
  RegisterOperation operation)
{
  for (int attempt{0}; attempt < kRegisterAttempts; ++attempt) {
    transport.write_batch({request});
    const auto replies = transport.collect(kMotorCount, feedback_deadline(parameters));
    for (const auto & received : replies) {
      if (received.frame.id != motor.mst_id()) {
        continue;
      }
      try {
        const RegisterReply reply{decode_register_reply(received.frame)};
        if (reply.motor_can_id == motor.esc_id() && reply.register_id == register_id &&
          reply.operation == operation)
        {
          return reply;
        }
      } catch (const std::invalid_argument &) {
      }
    }
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<float> read_float_register(
  CanTransport & transport,
  const DriverParameters & parameters,
  const DmMotor & motor,
  RegisterId register_id)
{
  const auto address = static_cast<std::uint8_t>(register_id);
  const auto reply = exchange_register(
    transport, parameters, motor,
    make_register_read(motor.esc_id(), address), address, RegisterOperation::read);
  if (!reply.has_value() || !std::isfinite(reply->float_value())) {
    return std::nullopt;
  }
  return reply->float_value();
}

void read_motor_limits(
  CanTransport & transport,
  const DriverParameters & parameters,
  DmMotor & motor,
  const StartupLogger & log)
{
  const auto position = read_float_register(
    transport, parameters, motor, RegisterId::position_max);
  const auto velocity = read_float_register(
    transport, parameters, motor, RegisterId::velocity_max);
  const auto torque = read_float_register(
    transport, parameters, motor, RegisterId::torque_max);
  if (!position.has_value() || !velocity.has_value() || !torque.has_value() ||
    *position <= 0.0F || *velocity <= 0.0F || *torque <= 0.0F)
  {
    log_safely(log, DriverLogLevel::warning,
      "motor limit read failed; using configured fallback limits");
    return;
  }

  const MotorLimits actual{*position, *velocity, *torque};
  motor.set_limits(actual);
  const MotorLimits & expected = parameters.limits_fallback;
  if (std::abs(actual.position_max - expected.position_max) > kLimitComparisonTolerance ||
    std::abs(actual.velocity_max - expected.velocity_max) > kLimitComparisonTolerance ||
    std::abs(actual.torque_max - expected.torque_max) > kLimitComparisonTolerance)
  {
    log_safely(log, DriverLogLevel::warning,
      "motor limits differ from configured fallback; using register values");
  }
}

void maybe_write_timeout(
  CanTransport & transport,
  const DriverParameters & parameters,
  const DmMotor & motor,
  const StartupLogger & log)
{
  if (!parameters.can.write_timeout_register) {
    return;
  }
  const auto timeout_counts = static_cast<std::uint32_t>(
    parameters.can.timeout_register_ms * kTimeoutCountsPerMillisecond);
  const auto address = static_cast<std::uint8_t>(RegisterId::timeout);
  const auto reply = exchange_register(
    transport, parameters, motor,
    make_register_write(motor.esc_id(), address, timeout_counts),
    address, RegisterOperation::write);
  if (!reply.has_value()) {
    log_safely(log, DriverLogLevel::warning,
      "TIMEOUT register write was not acknowledged");
  }
}

[[nodiscard]] std::array<std::optional<double>, kSwerveModuleCount> read_multi_turn_positions(
  CanTransport & transport,
  const DriverParameters & parameters,
  const std::array<DmMotor *, kMotorCount> & motors)
{
  std::array<std::optional<double>, kSwerveModuleCount> positions{};
  for (std::size_t index{0U}; index < positions.size(); ++index) {
    const auto value = read_float_register(
      transport, parameters, *motors[index], RegisterId::multi_turn_position);
    if (value.has_value()) {
      positions[index] = *value;
    }
  }
  return positions;
}

[[nodiscard]] FeedbackRouteResult poll_current_feedback(
  CanTransport & transport,
  const DriverParameters & parameters,
  const std::array<DmMotor *, kMotorCount> & motors,
  const StartupLogger & log)
{
  std::vector<CanFrame> neutral_commands;
  neutral_commands.reserve(motors.size());
  std::transform(
    motors.begin(), motors.end(), std::back_inserter(neutral_commands),
    [](const DmMotor * motor) {return motor->encode_command(MitCommand{});});
  transport.write_batch(neutral_commands);
  const auto frames = transport.collect(motors.size(), feedback_deadline(parameters));
  return route_feedback_frames(
    frames, motors,
    [&](const std::string & message) {
      log_safely(log, DriverLogLevel::warning, message);
    });
}

void seed_positions(
  const DriverParameters & parameters,
  const std::array<DmMotor *, kMotorCount> & motors,
  const std::array<std::optional<double>, kSwerveModuleCount> & multi_turn,
  const StartupLogger & log)
{
  for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
    DmMotor & steering = *motors[index];
    if (steering.health().has_feedback) {
      if (multi_turn[index].has_value()) {
        const bool consistent = steering.seed_position_from_multi_turn(
          steering.raw_position(), *multi_turn[index]);
        if (!consistent) {
          log_safely(log, DriverLogLevel::warning,
            "steering p_m disagrees with principal feedback; continuing with p_m seed");
        }
      } else {
        steering.seed_position(steering.raw_position());
        if (parameters.steering.gear_ratio > 1.0) {
          log_safely(log, DriverLogLevel::warning,
            "steering p_m unavailable; absolute module angle may be ambiguous");
        }
      }
    }

    DmMotor & drive = *motors[index + kSwerveModuleCount];
    if (drive.health().has_feedback) {
      drive.seed_position(drive.raw_position());
    }
  }
}

void enable_motors(
  CanTransport & transport,
  const DriverParameters & parameters,
  const std::array<DmMotor *, kMotorCount> & motors,
  const StartupLogger & log)
{
  const auto maximum_attempts = static_cast<std::size_t>(parameters.control.rate_hz);
  for (std::size_t attempt{0U}; attempt < std::max<std::size_t>(1U, maximum_attempts); ++attempt) {
    std::vector<CanFrame> enable_frames;
    enable_frames.reserve(motors.size());
    for (const DmMotor * motor : motors) {
      if (!motor->health().enabled()) {
        enable_frames.push_back(motor->special_command(SpecialCommand::enable));
      }
    }
    if (enable_frames.empty()) {
      return;
    }
    transport.write_batch(enable_frames);
    const auto frames = transport.collect(enable_frames.size(), feedback_deadline(parameters));
    static_cast<void>(route_feedback_frames(frames, motors, {}));
  }
  log_safely(log, DriverLogLevel::error,
    "one or more motors did not acknowledge enable; continuing in degraded mode");
}

}  // namespace

void initialize_motors(
  const DriverParameters & parameters,
  CanTransport & transport,
  const std::array<DmMotor *, kMotorCount> & motors,
  const StartupLogger & log)
{
  for (DmMotor * motor : motors) {
    read_motor_limits(transport, parameters, *motor, log);
    maybe_write_timeout(transport, parameters, *motor, log);
  }
  const auto multi_turn = read_multi_turn_positions(
    transport, parameters, motors);
  static_cast<void>(poll_current_feedback(transport, parameters, motors, log));
  seed_positions(parameters, motors, multi_turn, log);
  enable_motors(transport, parameters, motors, log);
  seed_positions(parameters, motors, multi_turn, log);
}

void disable_motors(
  CanTransport & transport,
  const std::array<DmMotor *, kMotorCount> & motors,
  const StartupLogger & log) noexcept
{
  try {
    std::vector<CanFrame> disable_frames;
    disable_frames.reserve(motors.size());
    std::transform(
      motors.begin(), motors.end(), std::back_inserter(disable_frames),
      [](const DmMotor * motor) {
        return motor->special_command(SpecialCommand::disable);
      });
    transport.write_batch(disable_frames);
  } catch (const std::exception & error) {
    log_safely(log, DriverLogLevel::warning,
      std::string{"failed to send disable batch: "} + error.what());
  } catch (...) {
    log_safely(log, DriverLogLevel::warning,
      "failed to send disable batch with an unknown error");
  }
}

}  // namespace dm_swerve_driver
