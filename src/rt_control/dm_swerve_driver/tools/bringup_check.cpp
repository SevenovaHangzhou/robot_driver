#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

#include "dm_swerve_driver/bringup_audit.hpp"
#include "dm_swerve_driver/dm_motor.hpp"
#include "dm_swerve_driver/socketcan_interface.hpp"

namespace {

using dm_swerve_driver::AuditMotor;
using dm_swerve_driver::BringupAuditOptions;
using dm_swerve_driver::CanFrame;
using dm_swerve_driver::DmMotor;
using dm_swerve_driver::DmMotorConfig;
using dm_swerve_driver::MotorError;
using dm_swerve_driver::MotorLimits;
using dm_swerve_driver::RegisterId;
using dm_swerve_driver::RegisterOperation;
using dm_swerve_driver::RegisterReply;
using dm_swerve_driver::SocketCanInterface;
using dm_swerve_driver::SocketCanOptions;

struct MotorAuditResult {
  AuditMotor motor{};
  MotorLimits limits{};
  std::optional<double> multi_turn_position;
  std::optional<dm_swerve_driver::MotorFeedback> feedback;
  int warnings{0};
};

[[nodiscard]] std::optional<RegisterReply> read_register(
  SocketCanInterface & can,
  const BringupAuditOptions & options,
  const AuditMotor & motor,
  RegisterId register_id)
{
  const auto address = static_cast<std::uint8_t>(register_id);
  for (int attempt{0}; attempt < options.retries; ++attempt) {
    can.write_batch({dm_swerve_driver::make_register_read(motor.esc_id, address)});
    const auto frames = can.collect(
      1U, std::chrono::steady_clock::now() + options.feedback_deadline);
    for (const auto & received : frames) {
      try {
        const RegisterReply reply{dm_swerve_driver::decode_register_reply(received.frame)};
        if (received.frame.id == motor.mst_id && reply.motor_can_id == motor.esc_id &&
          reply.register_id == address && reply.operation == RegisterOperation::read)
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
  SocketCanInterface & can,
  const BringupAuditOptions & options,
  const AuditMotor & motor,
  RegisterId register_id)
{
  const auto reply = read_register(can, options, motor, register_id);
  if (!reply.has_value() || !std::isfinite(reply->float_value())) {
    return std::nullopt;
  }
  return reply->float_value();
}

[[nodiscard]] std::optional<dm_swerve_driver::MotorFeedback> query_feedback(
  const SocketCanInterface & can,
  const BringupAuditOptions & options,
  const AuditMotor & motor,
  const MotorLimits & limits)
{
  for (int attempt{0}; attempt < options.retries; ++attempt) {
    can.write_batch({dm_swerve_driver::encode_mit_command(
        motor.esc_id, dm_swerve_driver::MitCommand{}, limits)});
    const auto replies = can.collect(
      1U, std::chrono::steady_clock::now() + options.feedback_deadline);
    for (const auto & reply : replies) {
      if (reply.frame.id != motor.mst_id) {
        continue;
      }
      try {
        return dm_swerve_driver::decode_motor_feedback(reply.frame, limits);
      } catch (const std::invalid_argument &) {
      }
    }
  }
  return std::nullopt;
}

[[nodiscard]] bool limits_match(const MotorLimits & actual, const MotorLimits & expected) noexcept
{
  constexpr double tolerance{1e-3};
  return std::abs(actual.position_max - expected.position_max) <= tolerance &&
         std::abs(actual.velocity_max - expected.velocity_max) <= tolerance &&
         std::abs(actual.torque_max - expected.torque_max) <= tolerance;
}

[[nodiscard]] MotorAuditResult audit_motor(
  SocketCanInterface & can,
  const BringupAuditOptions & options,
  const AuditMotor & motor)
{
  MotorAuditResult result;
  result.motor = motor;
  result.limits = options.expected_limits;
  const auto position = read_float_register(can, options, motor, RegisterId::position_max);
  const auto velocity = read_float_register(can, options, motor, RegisterId::velocity_max);
  const auto torque = read_float_register(can, options, motor, RegisterId::torque_max);
  if (position.has_value() && velocity.has_value() && torque.has_value() &&
    *position > 0.0F && *velocity > 0.0F && *torque > 0.0F)
  {
    result.limits = MotorLimits{*position, *velocity, *torque};
    result.warnings += limits_match(result.limits, options.expected_limits) ? 0 : 1;
  } else {
    ++result.warnings;
  }
  if (motor.steering) {
    const auto multi_turn = read_float_register(
      can, options, motor, RegisterId::multi_turn_position);
    if (multi_turn.has_value()) {
      result.multi_turn_position = *multi_turn;
    } else {
      ++result.warnings;
    }
  }
  result.feedback = query_feedback(can, options, motor, result.limits);
  if (!result.feedback.has_value()) {
    ++result.warnings;
  } else if (result.feedback->error != MotorError::disabled &&
    result.feedback->error != MotorError::enabled)
  {
    ++result.warnings;
  }
  if (result.feedback.has_value() && result.multi_turn_position.has_value() &&
    !DmMotor::multi_turn_consistent(
      result.feedback->position, *result.multi_turn_position))
  {
    ++result.warnings;
  }
  return result;
}

void print_result(const MotorAuditResult & result)
{
  std::cout << (result.motor.steering ? "STEER" : "DRIVE")
            << " ESC=0x" << std::hex << result.motor.esc_id
            << " MST=0x" << result.motor.mst_id << std::dec
            << " PMAX=" << result.limits.position_max
            << " VMAX=" << result.limits.velocity_max
            << " TMAX=" << result.limits.torque_max;
  if (result.feedback.has_value()) {
    std::cout << " ERR=0x" << std::hex
              << static_cast<unsigned int>(result.feedback->error) << std::dec
              << " POS=" << result.feedback->position
              << " MOS_C=" << static_cast<unsigned int>(result.feedback->mos_temperature_c)
              << " ROTOR_C=" << static_cast<unsigned int>(result.feedback->rotor_temperature_c);
  } else {
    std::cout << " FEEDBACK=MISSING";
  }
  if (result.multi_turn_position.has_value()) {
    std::cout << " P_M=" << *result.multi_turn_position;
  }
  std::cout << " WARNINGS=" << result.warnings << '\n';
}

[[nodiscard]] int run(const BringupAuditOptions & options)
{
  std::vector<std::uint16_t> receive_ids;
  receive_ids.reserve(options.motors.size());
  std::transform(
    options.motors.begin(), options.motors.end(), std::back_inserter(receive_ids),
    [](const AuditMotor & motor) {return motor.mst_id;});
  SocketCanInterface can{SocketCanOptions{
      options.interface_name, receive_ids, options.feedback_deadline, false}};
  int warnings{0};
  for (const auto & motor : options.motors) {
    const MotorAuditResult result{audit_motor(can, options, motor)};
    print_result(result);
    warnings += result.warnings;
  }
  std::cout << "AUDIT " << (warnings == 0 ? "COMPLETE" : "INCOMPLETE")
            << " warnings=" << warnings << '\n';
  return warnings == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> arguments;
  for (int index{1}; index < argc; ++index) {
    arguments.emplace_back(argv[index]);
  }
  if (std::find(arguments.begin(), arguments.end(), "--help") != arguments.end()) {
    std::cout << dm_swerve_driver::bringup_audit_usage();
    return 0;
  }
  try {
    return run(dm_swerve_driver::parse_bringup_audit_arguments(arguments));
  } catch (const std::invalid_argument & error) {
    std::cerr << "dm_swerve_bringup_check: " << error.what() << '\n'
              << dm_swerve_driver::bringup_audit_usage();
  } catch (const std::exception & error) {
    std::cerr << "dm_swerve_bringup_check: " << error.what() << '\n';
  }
  return 2;
}
