#include "gravity_ff_controller/control_core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace gravity_ff_controller
{
bool ControlCore::configure(const Limits & limits) noexcept
{
  const auto count = limits.maximum_effort_nm.size();
  if (count == 0U || limits.maximum_slew_nm_per_s.size() != count ||
    !std::isfinite(limits.fault_slew_nm_per_s) ||
    limits.fault_slew_nm_per_s <= 0.0)
  {
    return false;
  }
  for (std::size_t i = 0U; i < count; ++i) {
    if (!std::isfinite(limits.maximum_effort_nm[i]) ||
      limits.maximum_effort_nm[i] <= 0.0 ||
      !std::isfinite(limits.maximum_slew_nm_per_s[i]) ||
      limits.maximum_slew_nm_per_s[i] <= 0.0)
    {
      return false;
    }
  }
  limits_ = limits;
  output_nm_.assign(count, 0.0);
  desired_nm_.assign(count, 0.0);
  configured_ = true;
  activate();
  return true;
}

void ControlCore::activate() noexcept
{
  std::fill(output_nm_.begin(), output_nm_.end(), 0.0);
  std::fill(desired_nm_.begin(), desired_nm_.end(), 0.0);
  fault_latched_ = false;
  fault_ramp_active_ = false;
  last_valid_state_ = DriveState::kUnknown;
}

DriveState ControlCore::decode(const double raw) noexcept
{
  if (!std::isfinite(raw) || raw < 0.0 || raw > 65535.0 ||
    std::trunc(raw) != raw)
  {
    return DriveState::kUnknown;
  }
  const auto status = static_cast<std::uint16_t>(raw);
  if ((status & 0x004FU) == 0x0000U) {
    return DriveState::kNotReady;
  }
  if ((status & 0x004FU) == 0x0040U) {
    return DriveState::kSwitchOnDisabled;
  }
  if ((status & 0x006FU) == 0x0021U) {
    return DriveState::kReady;
  }
  if ((status & 0x006FU) == 0x0023U) {
    return DriveState::kSwitchedOn;
  }
  if ((status & 0x006FU) == 0x0027U) {
    return DriveState::kOperationEnabled;
  }
  if ((status & 0x006FU) == 0x0007U) {
    return DriveState::kQuickStop;
  }
  if ((status & 0x004FU) == 0x000FU) {
    return DriveState::kFaultReaction;
  }
  if ((status & 0x004FU) == 0x0008U) {
    return DriveState::kFault;
  }
  return DriveState::kUnknown;
}

void ControlCore::ramp_to(
  const std::vector<double> & desired,
  const double period) noexcept
{
  for (std::size_t i = 0U; i < output_nm_.size(); ++i) {
    const double change = limits_.maximum_slew_nm_per_s[i] * period;
    output_nm_[i] += std::clamp(desired[i] - output_nm_[i], -change, change);
  }
}

void ControlCore::fault_ramp(const double period) noexcept
{
  const double change = limits_.fault_slew_nm_per_s * period;
  for (double & value : output_nm_) {
    value += std::clamp(-value, -change, change);
  }
}

StepStatus ControlCore::step(
  const std::vector<double> & gravity,
  const std::vector<double> & scale,
  const std::vector<double> & status_words,
  const bool model_valid,
  const double period) noexcept
{
  if (!configured_) {
    return StepStatus::kInvalidInputLatched;
  }
  if (fault_latched_) {
    if (fault_ramp_active_) {
      fault_ramp(std::isfinite(period) && period > 0.0 ? period : 0.0);
    }
    return StepStatus::kAlreadyLatched;
  }
  bool valid = std::isfinite(period) && period > 0.0 &&
    gravity.size() == output_nm_.size() &&
    scale.size() == output_nm_.size() &&
    status_words.size() == output_nm_.size();
  bool any_enabled = false;
  bool any_fault_reaction = false;
  bool any_fault = false;
  bool all_terminal = status_words.size() == output_nm_.size();
  DriveState representative = DriveState::kUnknown;
  if (status_words.size() == output_nm_.size()) {
    for (std::size_t i = 0U; i < output_nm_.size(); ++i) {
      const auto state = decode(status_words[i]);
      valid = valid && state != DriveState::kUnknown;
      any_enabled = any_enabled || state == DriveState::kOperationEnabled;
      any_fault_reaction =
        any_fault_reaction || state == DriveState::kFaultReaction;
      any_fault = any_fault || state == DriveState::kFault;
      all_terminal = all_terminal &&
        (state == DriveState::kSwitchOnDisabled ||
        state == DriveState::kNotReady);
      representative = state;
    }
  }
  if (any_fault) {
    force_zero();
    fault_latched_ = true;
    fault_ramp_active_ = false;
    last_valid_state_ = DriveState::kFault;
    return StepStatus::kFaultLatched;
  }
  if (any_fault_reaction) {
    return StepStatus::kFaultReactionHold;
  }
  valid = valid && model_valid;
  if (gravity.size() == output_nm_.size() &&
    scale.size() == output_nm_.size())
  {
    for (std::size_t i = 0U; i < output_nm_.size(); ++i) {
      valid = valid && std::isfinite(gravity[i]) && std::isfinite(scale[i]) &&
        scale[i] >= 0.0 && scale[i] <= 1.0;
    }
  }
  if (!valid) {
    fault_latched_ = true;
    fault_ramp_active_ = last_valid_state_ == DriveState::kOperationEnabled;
    if (fault_ramp_active_) {
      fault_ramp(std::isfinite(period) && period > 0.0 ? period : 0.0);
    } else {
      force_zero();
    }
    return StepStatus::kInvalidInputLatched;
  }
  if (last_valid_state_ == DriveState::kOperationEnabled && !any_enabled &&
    !all_terminal)
  {
    return StepStatus::kDisableTransitionHold;
  }
  last_valid_state_ =
    any_enabled ? DriveState::kOperationEnabled : representative;
  for (std::size_t i = 0U; i < output_nm_.size(); ++i) {
    desired_nm_[i] =
      std::clamp(
      scale[i] * gravity[i], -limits_.maximum_effort_nm[i],
      limits_.maximum_effort_nm[i]);
  }
  ramp_to(desired_nm_, period);
  return StepStatus::kOk;
}

void ControlCore::force_zero() noexcept
{
  std::fill(output_nm_.begin(), output_nm_.end(), 0.0);
}
} // namespace gravity_ff_controller
