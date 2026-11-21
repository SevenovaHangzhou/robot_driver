// Motion groups (non-realtime configuration and controller plans): each group switches its own
// controllers (arms: trajectory/rolling; chassis: swerve/wheel trajectory) under the one enable
// state machine. Only one mode switch runs at a time (owner_ kMode); any group's failure, a
// disable or an emergency stop acts on the whole manager.
#include "ctrl_enable_manager/enable_manager_controller.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "robot_interfaces_qos/profiles.hpp"

namespace ctrl_enable_manager
{

bool EnableManagerController::configureMotionGroups()
{
  const auto names = get_node()->get_parameter("motion_groups").as_string_array();
  companion_controllers_ = get_node()->get_parameter("companion_controllers").as_string_array();
  if (names.size() > kMaxMotionGroups) {
    return false;
  }
  motion_group_count_ = names.size();
  managed_controllers_.clear();
  drive_switch_axis_.assign(joint_names_.size(), 0U);
  std::set<std::string> group_names;
  std::set<std::string> services;
  std::set<std::string> motion_joints;
  bool rolling_group = false;
  for (std::size_t index = 0U; index < motion_group_count_; ++index) {
    auto & group = motion_groups_[index];
    group.name = names[index];
    if (
      group.name.empty() || group.name.find_first_of("./") != std::string::npos ||
      !group_names.insert(group.name).second || !configureMotionGroup(group)) {
      return false;
    }
    managed_controllers_.insert(
      managed_controllers_.end(), group.controllers.begin(), group.controllers.end());
    // The rolling state topic is the arms' public one: a single rolling group.
    if (
      (group.mode_switching && !services.insert(group.service_name).second) ||
      (!group.rolling_controller.empty() && std::exchange(rolling_group, true))) {
      return false;
    }
    for (const auto & joint : group.joint_names) {
      if (!motion_joints.insert(joint).second) {
        return false;
      }
    }
    for (const auto axis : group.drive_axes) {
      if (std::exchange(drive_switch_axis_[axis], 1U) != 0U) {
        return false;
      }
    }
  }
  managed_controllers_.insert(
    managed_controllers_.end(), companion_controllers_.begin(), companion_controllers_.end());
  enable_only_ = managed_controllers_.empty();
  if (
    std::any_of(
      managed_controllers_.begin(), managed_controllers_.end(),
      [](const auto & name) { return name.empty(); }) ||
    std::set<std::string>(managed_controllers_.begin(), managed_controllers_.end()).size() !=
      managed_controllers_.size()) {
    return false;
  }
  // state_interface_configuration: every group's positions after the CiA402 axes, then the age.
  std::size_t offset = cia402_axes_.size();
  for (std::size_t index = 0U; index < motion_group_count_; ++index) {
    motion_groups_[index].state_offset = offset;
    offset += motion_groups_[index].joint_names.size();
  }
  motion_age_state_index_ = offset;
  return true;
}

bool EnableManagerController::configureMotionGroup(MotionGroup & group)
{
  const std::string prefix = group.name + ".";
  const auto parameter = [this, &prefix](const char * name) {
    return get_node()->get_parameter(prefix + name);
  };
  auto_declare<std::string>(prefix + "motion_controller", "");
  auto_declare<std::string>(prefix + "rolling_controller", "");
  auto_declare<std::string>(prefix + "trajectory_controller", "");
  auto_declare<std::string>(prefix + "mode_service", "");
  auto_declare<std::vector<std::string>>(prefix + "mode_switch_joints", {});
  auto_declare<std::vector<std::string>>(prefix + "motion_joints", {});
  auto_declare<int>(prefix + "mode_switch_maximum_sample_period_ms", 2);
  auto_declare<int>(prefix + "mode_switch_source_state_max_age_ms", 100);
  auto_declare<int>(prefix + "mode_switch_stable_interval_count", 5);
  auto_declare<int>(prefix + "mode_switch_timeout_ms", 500);
  auto_declare<double>(prefix + "mode_switch_feedback_age_limit_ms", 500.0);
  auto_declare<std::vector<double>>(
    prefix + "mode_switch_stable_velocity_thresholds",
    std::vector<double>(kDualArmAxisCount, 0.00872664626));
  auto_declare<std::vector<double>>(
    prefix + "mode_switch_takeover_tolerances",
    std::vector<double>(kDualArmAxisCount, 0.00872664626));

  group.default_controller = parameter("motion_controller").as_string();
  group.rolling_controller = parameter("rolling_controller").as_string();
  group.trajectory_controller = parameter("trajectory_controller").as_string();
  group.service_name = parameter("mode_service").as_string();
  group.controllers.clear();
  for (const auto & name :
       {group.default_controller, group.rolling_controller, group.trajectory_controller}) {
    if (!name.empty()) {
      group.controllers.push_back(name);
    }
  }
  group.mode_switching = !group.rolling_controller.empty() || !group.trajectory_controller.empty();
  group.default_mode =
    group.trajectory_controller.empty() ? ControlMode::kFjtReady : ControlMode::kVelocityReady;
  const bool controllers_valid =
    !group.default_controller.empty() &&
    (group.rolling_controller.empty() || group.trajectory_controller.empty()) &&
    (!group.mode_switching || !group.service_name.empty());

  group.joint_names = parameter("motion_joints").as_string_array();
  const auto maximum_sample_period_ms = parameter("mode_switch_maximum_sample_period_ms").as_int();
  const auto source_state_max_age_ms = parameter("mode_switch_source_state_max_age_ms").as_int();
  const auto stable_interval_count = parameter("mode_switch_stable_interval_count").as_int();
  const auto switch_timeout_ms = parameter("mode_switch_timeout_ms").as_int();
  const double feedback_age_limit_ms = parameter("mode_switch_feedback_age_limit_ms").as_double();
  const auto stable_velocity_thresholds =
    parameter("mode_switch_stable_velocity_thresholds").as_double_array();
  const auto takeover_tolerances = parameter("mode_switch_takeover_tolerances").as_double_array();

  std::vector<std::string> sorted_joints = group.joint_names;
  std::sort(sorted_joints.begin(), sorted_joints.end());
  const bool joints_valid =
    !group.joint_names.empty() && group.joint_names.size() <= kMaxMotionAxes &&
    std::adjacent_find(sorted_joints.begin(), sorted_joints.end()) == sorted_joints.end() &&
    std::all_of(group.joint_names.begin(), group.joint_names.end(), [this](const auto & name) {
      return std::find(joint_names_.begin(), joint_names_.end(), name) != joint_names_.end();
    });
  const auto valid_threshold_array = [&group](const std::vector<double> & values) {
    return values.size() == group.joint_names.size() &&
           std::all_of(values.begin(), values.end(), [](double value) {
             return std::isfinite(value) && value >= 0.0;
           });
  };
  const bool motion_parameters_valid =
    (!group.mode_switching && group.joint_names.empty()) ||
    (joints_valid && maximum_sample_period_ms > 0 && source_state_max_age_ms > 0 &&
     stable_interval_count > 0 && switch_timeout_ms > 0 && std::isfinite(feedback_age_limit_ms) &&
     feedback_age_limit_ms > 0.0 && valid_threshold_array(stable_velocity_thresholds) &&
     valid_threshold_array(takeover_tolerances));
  if (!controllers_valid || !motion_parameters_valid || !configureDriveModeSwitching(group)) {
    return false;
  }
  if (!group.joint_names.empty()) {
    group.maximum_sample_period_ns =
      static_cast<std::uint64_t>(maximum_sample_period_ms) * 1000000U;
    group.source_state_max_age_ns = static_cast<std::uint64_t>(source_state_max_age_ms) * 1000000U;
    group.stable_interval_count = static_cast<std::size_t>(stable_interval_count);
    group.switch_timeout = std::chrono::milliseconds(switch_timeout_ms);
    group.feedback_age_limit_ms = feedback_age_limit_ms;
    std::copy(
      stable_velocity_thresholds.begin(), stable_velocity_thresholds.end(),
      group.stable_velocity_thresholds.begin());
    std::copy(
      takeover_tolerances.begin(), takeover_tolerances.end(), group.takeover_tolerances.begin());
  }
  return true;
}

void EnableManagerController::createMotionGroupEndpoints(MotionGroup & group)
{
  group.callback_group.reset();
  group.standstill_service.reset();
  group.mode_service.reset();
  group.jtc_state_subscription.reset();
  group.rolling_state_subscription.reset();
  if (group.joint_names.empty()) {
    return;
  }
  group.callback_group =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  group.standstill_service = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/" + group.name + "/confirm_standstill",
    [this, &group](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handleConfirmStandstill(group, request, response);
    },
    rmw_qos_profile_services_default, group.callback_group);
  if (!group.mode_switching) {
    return;
  }
  group.mode_service = get_node()->create_service<ModeService>(
    group.service_name,
    [this, &group](
      const std::shared_ptr<ModeService::Request> request,
      std::shared_ptr<ModeService::Response> response) { handleSetMode(group, request, response); },
    rmw_qos_profile_services_default, group.callback_group);
  rclcpp::SubscriptionOptions options;
  options.callback_group = worker_callback_group_;
  const auto & trajectory =
    group.trajectory_controller.empty() ? group.default_controller : group.trajectory_controller;
  group.jtc_state_subscription =
    get_node()->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/" + trajectory + "/controller_state", robot_interfaces_qos::state(),
      [this, &group](const control_msgs::msg::JointTrajectoryControllerState::SharedPtr message) {
        handleJtcState(group, message);
      },
      options);
  if (!group.rolling_controller.empty()) {
    group.rolling_state_subscription =
      get_node()->create_subscription<robot_rt_control_interfaces::msg::RollingJointControlState>(
        "/rt/arms/rolling/state", robot_interfaces_qos::rolling_state(),
        [this, &group](
          const robot_rt_control_interfaces::msg::RollingJointControlState::SharedPtr message) {
          handleRollingState(group, message);
        },
        options);
  }
}

bool EnableManagerController::controllerRunsEnabled(const std::string & name) const
{
  // Enabled: every group in its default controller's mode (velocity for the chassis), and the
  // companions active.
  for (std::size_t index = 0U; index < motion_group_count_; ++index) {
    const auto & group = motion_groups_[index];
    if (
      std::find(group.controllers.begin(), group.controllers.end(), name) !=
      group.controllers.end()) {
      return controllerRunsInMode(group, name, group.default_mode);
    }
  }
  return true;
}

EnableManagerController::SwitchResult EnableManagerController::buildEnableSwitchRequest(
  const controller_manager_msgs::srv::ListControllers::Response & states, bool activate,
  controller_manager_msgs::srv::SwitchController::Request & request) const
{
  request.activate_controllers.clear();
  request.deactivate_controllers.clear();
  for (const auto & name : managed_controllers_) {
    const auto controller = std::find_if(
      states.controller.begin(), states.controller.end(),
      [&name](const auto & candidate) { return candidate.name == name; });
    if (controller == states.controller.end()) {
      return SwitchResult::kFailed;
    }
    const bool is_active = controller->state == "active";
    if (activate && controllerRunsEnabled(name)) {
      if (!is_active) {
        if (controller->state != "inactive") {
          return SwitchResult::kFailed;
        }
        request.activate_controllers.push_back(name);
      }
    } else if (is_active) {
      request.deactivate_controllers.push_back(name);
    }
  }
  return SwitchResult::kSuccess;
}

bool EnableManagerController::enableStateMatches(
  const controller_manager_msgs::srv::ListControllers::Response & states, bool activate) const
{
  for (const auto & name : managed_controllers_) {
    const auto controller = std::find_if(
      states.controller.begin(), states.controller.end(),
      [&name](const auto & candidate) { return candidate.name == name; });
    const char * expected = activate && controllerRunsEnabled(name) ? "active" : "inactive";
    if (controller == states.controller.end() || controller->state != expected) {
      return false;
    }
  }
  return true;
}

void EnableManagerController::storeGroupModes(ControlMode mode) noexcept
{
  for (std::size_t index = 0U; index < motion_group_count_; ++index) {
    motion_groups_[index].current_mode.store(mode, std::memory_order_release);
  }
}

void EnableManagerController::storeEnabledGroupModes() noexcept
{
  for (std::size_t index = 0U; index < motion_group_count_; ++index) {
    auto & group = motion_groups_[index];
    if (group.mode_switching) {
      group.current_mode.store(group.default_mode, std::memory_order_release);
    }
  }
}

void EnableManagerController::abortModeSwitches() noexcept
{
  for (std::size_t index = 0U; index < motion_group_count_; ++index) {
    motion_groups_[index].abort_requested.store(true, std::memory_order_release);
  }
}

}  // namespace ctrl_enable_manager
