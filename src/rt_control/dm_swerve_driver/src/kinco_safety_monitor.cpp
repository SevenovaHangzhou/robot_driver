#include "dm_swerve_driver/kinco_safety_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dm_swerve_driver {
namespace {

using SteadyClock = std::chrono::steady_clock;

[[nodiscard]] bool deadline_fresh(
  SteadyClock::time_point sample,
  SteadyClock::time_point now,
  double timeout_seconds) noexcept
{
  return sample > now || now - sample <= std::chrono::duration<double>{timeout_seconds};
}

[[nodiscard]] bool action_due(
  const std::optional<SteadyClock::time_point> & previous,
  SteadyClock::time_point now,
  double period_seconds) noexcept
{
  return !previous.has_value() || previous.value() > now ||
         now - previous.value() >= std::chrono::duration<double>{period_seconds};
}

[[nodiscard]] bool silent(
  const KincoAxisHealth & axis, std::uint64_t threshold) noexcept
{
  return axis.consecutive_missed_frames >= threshold;
}

[[nodiscard]] bool needs_automatic_recovery(
  const KincoAxisHealth & axis, std::uint64_t threshold) noexcept
{
  return silent(axis, threshold) ||
         (axis.has_feedback &&
         (axis.condition == KincoAxisCondition::disabled ||
         axis.condition == KincoAxisCondition::recoverable_fault));
}

}  // namespace

bool KincoAxisHealth::enabled() const noexcept
{
  return has_feedback && condition == KincoAxisCondition::enabled;
}

KincoSafetyMonitor::KincoSafetyMonitor(const DriverParameters & parameters)
: parameters_{parameters}
{
  validate_parameters(parameters_);
}

CommandDecision KincoSafetyMonitor::command_for_cycle(
  const ChassisSpeeds & command,
  std::optional<SteadyClock::time_point> command_timestamp,
  SteadyClock::time_point now)
{
  if (!std::isfinite(command.vx_mps) || !std::isfinite(command.vy_mps) ||
    !std::isfinite(command.omega_radps))
  {
    throw std::invalid_argument{"command watchdog input must be finite"};
  }
  const bool timed_out = !command_timestamp.has_value() || !deadline_fresh(
    command_timestamp.value(), now, parameters_.control.cmd_vel_timeout_s);
  const bool changed{timed_out != command_timed_out_};
  command_timed_out_ = timed_out;
  return CommandDecision{timed_out ? ChassisSpeeds{} : command, timed_out, changed};
}

YawDecision KincoSafetyMonitor::update_yaw(
  const std::optional<TimedYawSample> & imu,
  double wheel_delta_yaw_rad,
  SteadyClock::time_point now)
{
  if (!std::isfinite(wheel_delta_yaw_rad)) {
    throw std::invalid_argument{"wheel yaw delta must be finite"};
  }
  const bool imu_fresh = imu.has_value() && std::isfinite(imu->yaw_rad) &&
    deadline_fresh(imu->timestamp, now, parameters_.odometry.imu_timeout_s);
  const bool source_changed{imu_fresh == imu_fallback_};
  if (imu_fresh) {
    if (!yaw_initialized_) {
      yaw_rad_ = imu->yaw_rad;
      imu_offset_rad_ = 0.0;
      yaw_initialized_ = true;
    } else if (imu_fallback_) {
      imu_offset_rad_ = yaw_rad_ - imu->yaw_rad;
    } else {
      yaw_rad_ = imu_offset_rad_ + imu->yaw_rad;
    }
    imu_fallback_ = false;
  } else {
    if (!yaw_initialized_) {
      yaw_rad_ = 0.0;
      yaw_initialized_ = true;
    }
    yaw_rad_ += wheel_delta_yaw_rad;
    imu_fallback_ = true;
  }
  return YawDecision{yaw_rad_, imu_fallback_, source_changed};
}

KincoRecoveryActions KincoSafetyMonitor::recovery_actions(
  const std::array<KincoAxisHealth, kKincoAxisCount> & axes,
  SteadyClock::time_point now)
{
  KincoRecoveryActions actions{};
  bool current_fault{fault_latched_ || transport_faulted_ || steering_limit_faulted_};
  for (const auto & axis : axes) {
    current_fault = current_fault || silent(axis, parameters_.safety.feedback_silent_cycles) ||
      (axis.has_feedback && axis.condition != KincoAxisCondition::enabled);
    fault_latched_ = fault_latched_ ||
      (axis.has_feedback && axis.condition == KincoAxisCondition::latching_fault);
  }
  faulted_ = current_fault || fault_latched_;
  if (fault_latched_) {
    return actions;
  }

  for (std::size_t index{0U}; index < axes.size(); ++index) {
    const auto & axis = axes[index];
    if (!needs_automatic_recovery(axis, parameters_.safety.feedback_silent_cycles)) {
      continue;
    }
    if (recovery_attempts_[index] >= parameters_.safety.auto_recovery_limit) {
      fault_latched_ = true;
      break;
    }
    if (!action_due(last_recovery_[index], now, parameters_.safety.reenable_period_s)) {
      continue;
    }
    actions.clear_fault[index] =
      axis.has_feedback && axis.condition == KincoAxisCondition::recoverable_fault;
    actions.reenable[index] = true;
    last_recovery_[index] = now;
    ++recovery_attempts_[index];
  }
  if (fault_latched_) {
    actions = KincoRecoveryActions{};
    faulted_ = true;
  }
  return actions;
}

KincoRecoveryActions KincoSafetyMonitor::manual_clear_actions() noexcept
{
  KincoRecoveryActions actions;
  actions.clear_fault.fill(true);
  actions.reenable.fill(true);
  return actions;
}

bool KincoSafetyMonitor::complete_manual_clear(
  const std::array<KincoAxisHealth, kKincoAxisCount> & axes,
  const std::array<bool, kKincoAxisCount> & enable_confirmed) noexcept
{
  bool verified{!transport_faulted_ && !steering_limit_faulted_};
  for (std::size_t index{0U}; index < axes.size(); ++index) {
    verified = verified && enable_confirmed[index] && axes[index].enabled() &&
      axes[index].consecutive_missed_frames < parameters_.safety.feedback_silent_cycles;
  }
  if (!verified) {
    faulted_ = true;
    fault_latched_ = true;
    return false;
  }
  recovery_attempts_.fill(0U);
  last_recovery_.fill(std::nullopt);
  faulted_ = false;
  fault_latched_ = false;
  return true;
}

void KincoSafetyMonitor::restore_recovery_state(
  const std::array<std::uint32_t, kKincoAxisCount> & recovery_attempts,
  bool fault_latched) noexcept
{
  recovery_attempts_ = recovery_attempts;
  last_recovery_.fill(std::nullopt);
  fault_latched_ = fault_latched;
  faulted_ = fault_latched;
  transport_faulted_ = false;
  steering_limit_faulted_ = false;
}

void KincoSafetyMonitor::mark_transport_failure() noexcept
{
  transport_faulted_ = true;
  faulted_ = true;
}

void KincoSafetyMonitor::observe_feedback(
  const std::array<bool, kKincoAxisCount> & received) noexcept
{
  if (std::all_of(received.begin(), received.end(), [](bool value) {return value;})) {
    transport_faulted_ = false;
  }
}

bool KincoSafetyMonitor::observe_steering_limit_violation(bool active) noexcept
{
  const bool changed{active != steering_limit_faulted_};
  steering_limit_faulted_ = active;
  if (active) {
    faulted_ = true;
    fault_latched_ = true;
  }
  return changed;
}

bool KincoSafetyMonitor::faulted() const noexcept
{
  return faulted_ || transport_faulted_ || steering_limit_faulted_;
}

bool KincoSafetyMonitor::fault_latched() const noexcept
{
  return fault_latched_;
}

bool KincoSafetyMonitor::transport_faulted() const noexcept
{
  return transport_faulted_;
}

bool KincoSafetyMonitor::steering_limit_faulted() const noexcept
{
  return steering_limit_faulted_;
}

const std::array<std::uint32_t, kKincoAxisCount> &
KincoSafetyMonitor::recovery_attempts() const noexcept
{
  return recovery_attempts_;
}

}  // namespace dm_swerve_driver
