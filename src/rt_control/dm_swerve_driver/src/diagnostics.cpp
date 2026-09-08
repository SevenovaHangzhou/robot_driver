#include "dm_swerve_driver/diagnostics.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/key_value.hpp>

namespace dm_swerve_driver {
namespace {

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

const std::array<const char *, kMotorCount> kMotorNames{
  "front_left_steering", "front_right_steering",
  "rear_left_steering", "rear_right_steering",
  "front_left_drive", "front_right_drive",
  "rear_left_drive", "rear_right_drive"};

[[nodiscard]] std::uint16_t esc_id(
  const DriverParameters & parameters, std::size_t index) noexcept
{
  return index < kSwerveModuleCount ? parameters.motors.steering_esc_id[index] :
         parameters.motors.drive_esc_id[index - kSwerveModuleCount];
}

[[nodiscard]] const char * error_name(MotorError error) noexcept
{
  switch (error) {
    case MotorError::disabled: return "disabled";
    case MotorError::enabled: return "enabled";
    case MotorError::encoder: return "encoder error";
    case MotorError::encoder_read: return "encoder read error";
    case MotorError::over_voltage: return "over voltage";
    case MotorError::under_voltage: return "under voltage";
    case MotorError::over_current: return "over current";
    case MotorError::mos_over_temperature: return "MOS over temperature";
    case MotorError::coil_over_temperature: return "coil over temperature";
    case MotorError::communication_lost: return "communication lost";
    case MotorError::overload: return "overload";
  }
  return "unknown motor error";
}

[[nodiscard]] std::uint8_t motor_level(const DmMotorHealth & health) noexcept
{
  if (!health.has_feedback || health.error == MotorError::disabled ||
    health.error == MotorError::under_voltage ||
    health.error == MotorError::communication_lost)
  {
    return DiagnosticStatus::WARN;
  }
  return health.error == MotorError::enabled ? DiagnosticStatus::OK : DiagnosticStatus::ERROR;
}

void add_value(
  DiagnosticStatus & status, const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(std::move(item));
}

[[nodiscard]] DiagnosticStatus motor_status(
  const DmMotorHealth & health,
  const DriverParameters & parameters,
  std::size_t index)
{
  DiagnosticStatus result;
  result.name = std::string{"dm_swerve_driver/"} + kMotorNames[index];
  result.hardware_id = "esc_" + std::to_string(esc_id(parameters, index));
  result.level = motor_level(health);
  result.message = health.has_feedback ? error_name(health.error) : "no feedback received";
  add_value(result, "error_code", std::to_string(static_cast<std::uint8_t>(health.error)));
  add_value(result, "mos_temperature_c", std::to_string(health.mos_temperature_c));
  add_value(result, "rotor_temperature_c", std::to_string(health.rotor_temperature_c));
  add_value(result, "missed_frames", std::to_string(health.missed_frames));
  add_value(result, "consecutive_missed_frames",
    std::to_string(health.consecutive_missed_frames));
  add_value(result, "seeded_from_multi_turn",
    health.seeded_from_multi_turn ? "true" : "false");
  return result;
}

[[nodiscard]] const char * summary_message(
  const ControlLoopStatus & status, bool motor_error, bool degraded) noexcept
{
  if (status.fault_latched) {
    return "motor fault latched; manual clear required";
  }
  if (status.faulted) {
    return status.transport_faulted ?
      "transport faulted; all drive commands gated" :
      "motor safety faulted; all drive commands gated";
  }
  if (status.bus_silent) {
    return "CAN bus silent; drive commands gated";
  }
  if (motor_error) {
    return "one or more motors report faults";
  }
  return degraded ? "driver running with recoverable degradation" : "driver healthy";
}

[[nodiscard]] DiagnosticStatus summary_status(
  const ControlLoopStatus & status,
  const std::vector<DiagnosticStatus> & motor_statuses)
{
  DiagnosticStatus summary;
  summary.name = "dm_swerve_driver/summary";
  summary.hardware_id = "swerve_chassis";
  const bool motor_error = std::any_of(
    motor_statuses.begin(), motor_statuses.end(), [](const DiagnosticStatus & motor) {
      return motor.level == DiagnosticStatus::ERROR;
    });
  const bool degraded = !status.initialized || status.command_timed_out || status.imu_fallback ||
    status.unknown_frames_last_cycle != 0U ||
    status.rejected_frames_last_cycle != 0U ||
    status.stale_frames_last_cycle != 0U;
  summary.level = status.faulted || status.fault_latched || status.bus_silent || motor_error ?
    DiagnosticStatus::ERROR :
    (degraded ? DiagnosticStatus::WARN : DiagnosticStatus::OK);
  summary.message = summary_message(status, motor_error, degraded);
  add_value(summary, "command_timed_out", status.command_timed_out ? "true" : "false");
  add_value(summary, "imu_fallback", status.imu_fallback ? "true" : "false");
  add_value(summary, "completed_cycles", std::to_string(status.completed_cycles));
  add_value(summary, "loop_overruns", std::to_string(status.loop_overruns));
  add_value(summary, "unknown_frames", std::to_string(status.unknown_frames));
  add_value(summary, "rejected_frames", std::to_string(status.rejected_frames));
  add_value(summary, "stale_frames", std::to_string(status.stale_frames));
  add_value(summary, "faulted", status.faulted ? "true" : "false");
  add_value(summary, "fault_latched", status.fault_latched ? "true" : "false");
  add_value(summary, "transport_faulted", status.transport_faulted ? "true" : "false");
  std::uint64_t recovery_attempts{0U};
  for (const auto attempts : status.recovery_attempts) {
    recovery_attempts += attempts;
  }
  add_value(summary, "recovery_attempts", std::to_string(recovery_attempts));
  add_value(summary, "unknown_frames_last_cycle",
    std::to_string(status.unknown_frames_last_cycle));
  add_value(summary, "rejected_frames_last_cycle",
    std::to_string(status.rejected_frames_last_cycle));
  add_value(summary, "stale_frames_last_cycle",
    std::to_string(status.stale_frames_last_cycle));
  return summary;
}

}  // namespace

std::vector<DiagnosticStatus> build_diagnostic_statuses(
  const ControlLoopStatus & status,
  const DriverParameters & parameters)
{
  std::vector<DiagnosticStatus> diagnostics;
  diagnostics.reserve(kMotorCount + 1U);
  for (std::size_t index{0U}; index < status.motors.size(); ++index) {
    diagnostics.push_back(motor_status(status.motors[index], parameters, index));
  }
  diagnostics.push_back(summary_status(status, diagnostics));
  return diagnostics;
}

}  // namespace dm_swerve_driver
