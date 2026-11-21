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
  return modeConfirmed(axis) ||
         (!motion_group_.drive_axis.empty() && motion_group_.drive_axis[axis] != 0U &&
          actual_modes_[axis] == kCspMode);
}

bool EnableManagerController::configureDriveModeSwitching(MotionGroup & group)
{
  group.drive_joint_names = get_node()->get_parameter("mode_switch_joints").as_string_array();
  group.drive_axes.clear();
  group.drive_axis.assign(joint_names_.size(), 0U);
  if (group.drive_joint_names.empty()) {
    group.drive_axis.clear();
    return true;
  }
  // Only a velocity-mode group switches drive modes, and only drives with a fixed startup mode
  // other than CSP that the standstill check covers.
  if (
    group.trajectory_controller.empty() || expected_modes_.empty() ||
    group.drive_joint_names.size() > kMaxMotionAxes) {
    return false;
  }
  for (const auto & name : group.drive_joint_names) {
    const auto joint = std::find(joint_names_.begin(), joint_names_.end(), name);
    if (
      joint == joint_names_.end() ||
      std::find(group.joint_names.begin(), group.joint_names.end(), name) ==
        group.joint_names.end()) {
      return false;
    }
    const auto axis = static_cast<std::size_t>(std::distance(joint_names_.begin(), joint));
    if (group.drive_axis[axis] != 0U || expected_modes_[axis] == kCspMode) {
      return false;
    }
    group.drive_axis[axis] = 1U;
    group.drive_axes.push_back(axis);
  }
  return true;
}

bool EnableManagerController::bindDriveModeCommands(MotionGroup & group)
{
  group.drive_command_offset = cia402_axes_.size() + handshake_devices_.size() * 2U;
  for (std::size_t index = 0U; index < group.drive_joint_names.size(); ++index) {
    if (
      command_interfaces_[group.drive_command_offset + index].get_name() !=
      group.drive_joint_names[index] + "/mode_of_operation") {
      return false;
    }
  }
  group.drive_csp_requested.store(false, std::memory_order_release);
  for (auto & display : group.drive_mode_display) {
    display.store(256, std::memory_order_relaxed);
  }
  return true;
}

void EnableManagerController::writeDriveModes(MotionGroup & group) noexcept
{
  if (group.drive_axes.empty()) {
    return;
  }
  if (phase_.load(std::memory_order_acquire) != Phase::kEnabled) {
    group.drive_csp_requested.store(false, std::memory_order_release);
  }
  const bool csp = group.drive_csp_requested.load(std::memory_order_acquire);
  for (std::size_t index = 0U; index < group.drive_axes.size(); ++index) {
    const std::size_t axis = group.drive_axes[index];
    (void)command_interfaces_[group.drive_command_offset + index].set_value(
      static_cast<double>(csp ? kCspMode : expected_modes_[axis]));
    group.drive_mode_display[index].store(actual_modes_[axis], std::memory_order_release);
  }
}

bool EnableManagerController::switchedDrivesUseCsp(
  const MotionGroup & group, ControlMode mode) const noexcept
{
  return mode == ControlMode::kFjtReady && group.default_mode == ControlMode::kVelocityReady;
}

bool EnableManagerController::switchDriveModes(MotionGroup & group, ControlMode mode)
{
  if (group.drive_axes.empty()) {
    return true;
  }
  const bool csp = switchedDrivesUseCsp(group, mode);
  group.drive_csp_requested.store(csp, std::memory_order_release);
  const auto deadline = std::chrono::steady_clock::now() + group.switch_timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (
      owner_.load(std::memory_order_acquire) != Owner::kMode ||
      phase_.load(std::memory_order_acquire) != Phase::kEnabled) {
      return false;
    }
    bool confirmed = true;
    for (std::size_t index = 0U; index < group.drive_axes.size(); ++index) {
      const std::int16_t expected = csp ? kCspMode : expected_modes_[group.drive_axes[index]];
      confirmed =
        confirmed && group.drive_mode_display[index].load(std::memory_order_acquire) == expected;
    }
    if (confirmed) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

}  // namespace ctrl_enable_manager
