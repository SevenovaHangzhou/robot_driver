#include "dm_swerve_driver/steering_rezero.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dm_swerve_driver/feedback_router.hpp"

namespace dm_swerve_driver {
namespace {

using SteeringMotors = std::array<DmMotor, kSwerveModuleCount>;

[[nodiscard]] SteeringMotors make_steering_motors(
  const DriverParameters & parameters)
{
  return {
    DmMotor{steering_motor_config(parameters, 0U)},
    DmMotor{steering_motor_config(parameters, 1U)},
    DmMotor{steering_motor_config(parameters, 2U)},
    DmMotor{steering_motor_config(parameters, 3U)}};
}

[[nodiscard]] std::array<DmMotor *, kMotorCount> motor_pointers(
  SteeringMotors & motors) noexcept
{
  std::array<DmMotor *, kMotorCount> pointers{};
  for (std::size_t index{0U}; index < motors.size(); ++index) {
    pointers[index] = &motors[index];
  }
  return pointers;
}

[[nodiscard]] FeedbackRouteResult exchange(
  CanTransport & transport,
  const DriverParameters & parameters,
  const std::vector<CanFrame> & commands,
  const std::array<DmMotor *, kMotorCount> & motors)
{
  const auto command_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch());
  transport.write_batch(commands);
  const auto feedback = transport.collect(
    commands.size(), std::chrono::steady_clock::now() +
    std::chrono::microseconds{parameters.can.feedback_deadline_us});
  return route_feedback_frames(feedback, motors, {}, command_timestamp);
}

[[nodiscard]] const char * failure_reason(
  SteeringRezeroFailureReason reason) noexcept
{
  switch (reason) {
    case SteeringRezeroFailureReason::save_zero_acknowledgement_missing:
      return "save_zero acknowledgement missing";
    case SteeringRezeroFailureReason::verification_feedback_missing:
      return "verification feedback missing";
    case SteeringRezeroFailureReason::position_out_of_tolerance:
      return "verified position outside tolerance";
  }
  return "unknown verification failure";
}

}  // namespace

SteeringRezeroResult perform_steering_rezero(
  const DriverParameters & parameters,
  CanTransport & transport)
{
  validate_parameters(parameters);
  if (!transport.is_open()) {
    throw std::logic_error{"steering rezero requires an open CAN transport"};
  }

  auto motors = make_steering_motors(parameters);
  const auto pointers = motor_pointers(motors);
  std::vector<CanFrame> save_commands;
  std::vector<CanFrame> verification_commands;
  save_commands.reserve(motors.size());
  verification_commands.reserve(motors.size());
  for (const auto & motor : motors) {
    save_commands.push_back(motor.special_command(SpecialCommand::save_zero));
    verification_commands.push_back(motor.encode_command(MitCommand{}));
  }

  const auto acknowledgements = exchange(
    transport, parameters, save_commands, pointers);
  const auto verification = exchange(
    transport, parameters, verification_commands, pointers);
  SteeringRezeroResult result;
  for (std::size_t index{0U}; index < motors.size(); ++index) {
    if (!acknowledgements.received[index]) {
      result.failures.push_back(SteeringRezeroFailure{
          motors[index].esc_id(),
          SteeringRezeroFailureReason::save_zero_acknowledgement_missing});
    } else if (!verification.received[index]) {
      result.failures.push_back(SteeringRezeroFailure{
          motors[index].esc_id(),
          SteeringRezeroFailureReason::verification_feedback_missing});
    } else if (std::abs(motors[index].raw_position()) >
      parameters.steering.rezero_tolerance_rad)
    {
      result.failures.push_back(SteeringRezeroFailure{
          motors[index].esc_id(),
          SteeringRezeroFailureReason::position_out_of_tolerance});
    }
  }
  return result;
}

std::string steering_rezero_failure_message(const SteeringRezeroResult & result)
{
  std::ostringstream message;
  message << "steering rezero failed for ";
  for (std::size_t index{0U}; index < result.failures.size(); ++index) {
    if (index != 0U) {
      message << ", ";
    }
    const auto & failure = result.failures[index];
    message << "ESC_ID " << failure.esc_id << " (" <<
      failure_reason(failure.reason) << ')';
  }
  return message.str();
}

}  // namespace dm_swerve_driver
