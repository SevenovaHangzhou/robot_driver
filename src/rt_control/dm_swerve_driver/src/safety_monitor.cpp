#include "dm_swerve_driver/safety_monitor.hpp"

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

}  // namespace

SafetyMonitor::SafetyMonitor(const DriverParameters & parameters)
: parameters_{parameters}
{
  validate_parameters(parameters_);
}

CommandDecision SafetyMonitor::command_for_cycle(
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

YawDecision SafetyMonitor::update_yaw(
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

RecoveryActions SafetyMonitor::recovery_actions(
  const std::array<DmMotorHealth, kMotorCount> & motors,
  SteadyClock::time_point now)
{
  RecoveryActions actions{};
  for (std::size_t index{0U}; index < motors.size(); ++index) {
    const auto & motor = motors[index];
    const bool silent = motor.consecutive_missed_frames >=
      parameters_.safety.feedback_silent_cycles;
    const bool not_enabled = motor.has_feedback && motor.error != MotorError::enabled;
    if (motor.has_fault() && action_due(
        last_clear_fault_[index], now, parameters_.safety.reenable_period_s))
    {
      actions.clear_fault[index] = true;
      last_clear_fault_[index] = now;
    }
    if ((silent || not_enabled) && action_due(
        last_reenable_[index], now, parameters_.safety.reenable_period_s))
    {
      actions.reenable[index] = true;
      last_reenable_[index] = now;
    }
  }
  return actions;
}

bool SafetyMonitor::all_bus_silent(
  const std::array<DmMotorHealth, kMotorCount> & motors) const noexcept
{
  return std::all_of(motors.begin(), motors.end(), [&](const DmMotorHealth & motor) {
      return motor.consecutive_missed_frames >= parameters_.safety.feedback_silent_cycles;
    });
}

}  // namespace dm_swerve_driver
