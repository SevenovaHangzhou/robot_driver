// Drive operation modes: the mode_of_operation_display readback of each managed axis, and the
// mode_of_operation command of the drives whose mode follows the controller set (the swerve
// drive wheels: CSV under the swerve controller, CSP under the wheel trajectory controller).
#include "ctrl_enable_manager/enable_manager_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

namespace ctrl_enable_manager
{

bool EnableManagerController::modeConfirmed(std::size_t axis) const noexcept
{
  return expected_modes_.empty() || actual_modes_[axis] == expected_modes_[axis];
}

bool EnableManagerController::modePending(std::size_t axis) const noexcept
{
  return !expected_modes_.empty() && actual_modes_[axis] == 0;
}

void EnableManagerController::refreshModes()
{
  for (std::size_t axis = 0; axis < expected_modes_.size(); ++axis) {
    const double value = state_interfaces_[mode_state_indices_[axis]].get_value();
    actual_modes_[axis] =
      std::isfinite(value) && value >= -128.0 && value <= 127.0 && std::floor(value) == value
        ? static_cast<std::int16_t>(value)
        : 256;
  }
}

bool EnableManagerController::runningModeConfirmed(std::size_t axis) const noexcept
{
  // While enabled a switched drive may be in its startup mode or in CSP; the mode switch itself
  // confirms the mode its controller set needs.
  return modeConfirmed(axis) || (!mode_switch_axis_.empty() && mode_switch_axis_[axis] != 0U &&
                                 actual_modes_[axis] == kCspMode);
}

bool EnableManagerController::configureDriveModeSwitching()
{
  mode_switch_joint_names_ = get_node()->get_parameter("mode_switch_joints").as_string_array();
  mode_switch_axes_.clear();
  mode_switch_axis_.assign(joint_names_.size(), 0U);
  if (mode_switch_joint_names_.empty()) {
    mode_switch_axis_.clear();
    return true;
  }
  // Only a velocity-mode group switches drive modes, and only drives with a fixed startup mode
  // other than CSP that the standstill check covers.
  if (
    trajectory_motion_controller_.empty() || expected_modes_.empty() ||
    mode_switch_joint_names_.size() > kMaxMotionAxes) {
    return false;
  }
  for (const auto & name : mode_switch_joint_names_) {
    const auto joint = std::find(joint_names_.begin(), joint_names_.end(), name);
    if (
      joint == joint_names_.end() ||
      std::find(motion_joint_names_.begin(), motion_joint_names_.end(), name) ==
        motion_joint_names_.end()) {
      return false;
    }
    const auto axis = static_cast<std::size_t>(std::distance(joint_names_.begin(), joint));
    if (mode_switch_axis_[axis] != 0U || expected_modes_[axis] == kCspMode) {
      return false;
    }
    mode_switch_axis_[axis] = 1U;
    mode_switch_axes_.push_back(axis);
  }
  return true;
}

bool EnableManagerController::bindDriveModeCommands()
{
  drive_mode_command_offset_ = cia402_axes_.size() + handshake_devices_.size() * 2U;
  for (std::size_t index = 0U; index < mode_switch_joint_names_.size(); ++index) {
    if (
      command_interfaces_[drive_mode_command_offset_ + index].get_name() !=
      mode_switch_joint_names_[index] + "/mode_of_operation") {
      return false;
    }
  }
  drive_csp_requested_.store(false, std::memory_order_release);
  for (auto & display : drive_mode_display_) {
    display.store(256, std::memory_order_relaxed);
  }
  return true;
}

void EnableManagerController::writeDriveModes() noexcept
{
  if (mode_switch_axes_.empty()) {
    return;
  }
  if (phase_.load(std::memory_order_acquire) != Phase::kEnabled) {
    drive_csp_requested_.store(false, std::memory_order_release);
  }
  const bool csp = drive_csp_requested_.load(std::memory_order_acquire);
  for (std::size_t index = 0U; index < mode_switch_axes_.size(); ++index) {
    const std::size_t axis = mode_switch_axes_[index];
    (void)command_interfaces_[drive_mode_command_offset_ + index].set_value(
      static_cast<double>(csp ? kCspMode : expected_modes_[axis]));
    drive_mode_display_[index].store(actual_modes_[axis], std::memory_order_release);
  }
}

bool EnableManagerController::switchedDrivesUseCsp(ControlMode mode) const noexcept
{
  return mode == ControlMode::kFjtReady && motion_controller_mode_ == ControlMode::kVelocityReady;
}

bool EnableManagerController::switchDriveModes(ControlMode mode)
{
  if (mode_switch_axes_.empty()) {
    return true;
  }
  const bool csp = switchedDrivesUseCsp(mode);
  drive_csp_requested_.store(csp, std::memory_order_release);
  const auto deadline = std::chrono::steady_clock::now() + mode_switch_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    if (
      owner_.load(std::memory_order_acquire) != Owner::kMode ||
      phase_.load(std::memory_order_acquire) != Phase::kEnabled) {
      return false;
    }
    bool confirmed = true;
    for (std::size_t index = 0U; index < mode_switch_axes_.size(); ++index) {
      const std::int16_t expected = csp ? kCspMode : expected_modes_[mode_switch_axes_[index]];
      confirmed =
        confirmed && drive_mode_display_[index].load(std::memory_order_acquire) == expected;
    }
    if (confirmed) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

}  // namespace ctrl_enable_manager
