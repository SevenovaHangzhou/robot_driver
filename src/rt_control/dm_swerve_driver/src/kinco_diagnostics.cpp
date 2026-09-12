#include "dm_swerve_driver/diagnostics.hpp"
#include <algorithm>
#include <string>

namespace dm_swerve_driver {
namespace {
using Status = diagnostic_msgs::msg::DiagnosticStatus;
constexpr std::array<const char *, kSwerveModuleCount> names{
  "front_left", "front_right", "rear_left", "rear_right"};
void add(Status & status, const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue field;
  field.key = key;
  field.value = value;
  status.values.push_back(field);
}
}

std::vector<Status> build_kinco_diagnostic_statuses(const ControlLoopStatus & state)
{
  std::vector<Status> result;
  result.reserve(13U);
  for (std::size_t i{0U}; i < kMotorCount; ++i) {
    const auto & axis = state.ethercat.feedback[i];
    Status status;
    status.name = std::string{"dm_swerve_driver/"} + names[i % kSwerveModuleCount] +
      (i < kSwerveModuleCount ? "_steering" : "_drive");
    status.hardware_id = "kinco_axis_" + std::to_string(i);
    const auto decoded = decode_ds402_state(axis.status_word);
    const auto fault = classify_kinco_error_word(axis.error_word);
    status.level = !axis.online || !state.ethercat.domain.healthy() ||
      decoded == Ds402State::fault || decoded == Ds402State::fault_reaction_active ||
      decoded == Ds402State::unknown ||
      axis.extended_error_word != 0U || fault.disposition == FaultDisposition::latch ?
      Status::ERROR : (decoded == Ds402State::operation_enabled &&
      fault.disposition == FaultDisposition::none ? Status::OK : Status::WARN);
    status.message = status.level == Status::OK ? "operation enabled" : "axis unavailable or faulted";
    add(status, "statusword", std::to_string(axis.status_word));
    add(status, "mode_display", std::to_string(axis.mode_display));
    add(status, "error_2601", std::to_string(axis.error_word));
    add(status, "error_2602", std::to_string(axis.extended_error_word));
    add(status, "recovery_attempts", std::to_string(state.recovery_attempts[i]));
    result.push_back(status);
  }
  for (std::size_t i{0U}; i < kSwerveModuleCount; ++i) {
    const auto & source = state.steering_sources[i];
    Status status;
    status.name = std::string{"dm_swerve_driver/"} + names[i] + "_encoder";
    status.hardware_id = "steering_encoder_" + std::to_string(i);
    status.level = !source.valid || source.disagreement ? Status::ERROR :
      (source.degraded || !state.encoder_heartbeat[i] ? Status::WARN : Status::OK);
    status.message = source.source == SteeringAngleSource::external_encoder ? "external absolute encoder" :
      (source.source == SteeringAngleSource::motor_backup ? "motor encoder backup" : "no valid steering source");
    add(status, "angle_rad", std::to_string(source.angle_rad));
    add(status, "heartbeat_fresh", state.encoder_heartbeat[i] ? "true" : "false");
    add(status, "nmt_state", std::to_string(state.encoder_nmt_state[i]));
    add(status, "slipping", state.slipping_modules[i] ? "true" : "false");
    result.push_back(status);
  }
  Status summary;
  summary.name = "dm_swerve_driver/summary";
  summary.hardware_id = "kinco_ethercat_chassis";
  summary.level = state.faulted || state.fault_latched || !state.ethercat.domain.healthy() ?
    Status::ERROR : Status::OK;
  for (const auto & status : result) {summary.level = std::max(summary.level, status.level);}
  if (summary.level == Status::OK && (state.slip_detected || state.imu_fallback || state.command_timed_out)) {
    summary.level = Status::WARN;
  }
  summary.message = state.fault_latched ? "fault latched; manual clear required" :
    (summary.level == Status::OK ? "Kinco driver healthy" : "Kinco driver degraded or stopped");
  add(summary, "working_counter", std::to_string(state.ethercat.domain.working_counter));
  add(summary, "domain_complete", state.ethercat.domain.healthy() ? "true" : "false");
  add(summary, "all_slaves_operational", state.ethercat.domain.all_slaves_operational ? "true" : "false");
  add(summary, "link_up", state.ethercat.domain.link_up ? "true" : "false");
  add(summary, "loop_overruns", std::to_string(state.loop_overruns));
  add(summary, "slip_detected", state.slip_detected ? "true" : "false");
  result.push_back(summary);
  return result;
}
}  // namespace dm_swerve_driver
