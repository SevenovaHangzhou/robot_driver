#include "enable_manager/enable_manager_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/joint_control_mode.hpp"
#include "robot_rt_control_interfaces/msg/rolling_service_result.hpp"
#include "robot_system_interfaces/msg/error_code.hpp"

namespace enable_manager
{

const std::array<const char *, EnableManagerController::kAxisCount>
EnableManagerController::kJointNames = {
  "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5",
  "right_joint6", "left_joint1", "left_joint2", "left_joint3", "left_joint4",
  "left_joint5", "left_joint6", "turn", "updown"};

const std::array<std::array<std::int8_t, 3>, EnableManagerController::kBatchCount>
EnableManagerController::kEnableBatches = {{{0, 1, 2}, {6, 7, 8}, {3, 4, 5}, {9, 10, 11},
  {12, 13, -1}}};

const std::array<std::uint8_t, EnableManagerController::kBatchCount>
EnableManagerController::kBatchSizes = {3U, 3U, 3U, 3U, 2U};

controller_interface::CallbackReturn EnableManagerController::on_init()
{
  auto_declare<double>("batch_timeout", 4.0);
  auto_declare<double>("disable_stage_timeout", 4.0);
  auto_declare<double>("inter_batch_delay", 0.2);
  auto_declare<double>("fault_reset_timeout", 4.0);
  auto_declare<double>("controller_switch_timeout", 4.0);
  auto_declare<int>("service_result_timeout_ms", 30000);
  auto_declare<std::vector<std::string>>(
    "motion_controller_names", {"whole_body_jtc", "rolling_trajectory_controller"});
  auto_declare<std::string>("default_motion_controller", "whole_body_jtc");
  auto_declare<std::string>(
    "rolling_motion_controller", "rolling_trajectory_controller");
  auto_declare<int>("mode_switch_maximum_sample_period_ms", 8);
  auto_declare<int>("mode_switch_source_state_max_age_ms", 100);
  auto_declare<int>("mode_switch_stable_interval_count", 5);
  auto_declare<int>("mode_switch_timeout_ms", 500);
  auto_declare<double>("mode_switch_feedback_age_limit_ms", 500.0);
  auto_declare<std::vector<double>>(
    "mode_switch_stable_velocity_thresholds",
    {0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
      0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
      0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
      0.00872664626, 0.001});
  auto_declare<std::vector<double>>(
    "mode_switch_takeover_tolerances",
    {0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
      0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
      0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
      0.00872664626, 0.005});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
EnableManagerController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(kAxisCount);
  for (const char * joint : kJointNames) {
    configuration.names.emplace_back(std::string(joint) + "/control_word");
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
EnableManagerController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(kAxisCount * 2U + 1U);
  for (const char * joint : kJointNames) {
    configuration.names.emplace_back(std::string(joint) + "/status_word");
  }
  for (const char * joint : kJointNames) {
    configuration.names.emplace_back(std::string(joint) + "/position");
  }
  configuration.names.emplace_back("ethercat_domain/process_data_age_ms");
  return configuration;
}

controller_interface::CallbackReturn EnableManagerController::on_configure(
  const rclcpp_lifecycle::State &)
{
  batch_timeout_seconds_ = get_node()->get_parameter("batch_timeout").as_double();
  disable_stage_timeout_seconds_ =
    get_node()->get_parameter("disable_stage_timeout").as_double();
  inter_batch_delay_seconds_ = get_node()->get_parameter("inter_batch_delay").as_double();
  fault_reset_timeout_seconds_ = get_node()->get_parameter("fault_reset_timeout").as_double();
  controller_switch_timeout_seconds_ =
    get_node()->get_parameter("controller_switch_timeout").as_double();
  const auto service_timeout = get_node()->get_parameter("service_result_timeout_ms").as_int();
  motion_controller_names_ =
    get_node()->get_parameter("motion_controller_names").as_string_array();
  default_motion_controller_ =
    get_node()->get_parameter("default_motion_controller").as_string();
  rolling_motion_controller_ =
    get_node()->get_parameter("rolling_motion_controller").as_string();
  const auto maximum_sample_period_ms =
    get_node()->get_parameter("mode_switch_maximum_sample_period_ms").as_int();
  const auto source_state_max_age_ms =
    get_node()->get_parameter("mode_switch_source_state_max_age_ms").as_int();
  const auto stable_interval_count =
    get_node()->get_parameter("mode_switch_stable_interval_count").as_int();
  const auto mode_switch_timeout_ms =
    get_node()->get_parameter("mode_switch_timeout_ms").as_int();
  const double mode_switch_feedback_age_limit_ms =
    get_node()->get_parameter("mode_switch_feedback_age_limit_ms").as_double();
  const auto stable_velocity_thresholds =
    get_node()->get_parameter("mode_switch_stable_velocity_thresholds").as_double_array();
  const auto takeover_tolerances =
    get_node()->get_parameter("mode_switch_takeover_tolerances").as_double_array();

  std::vector<std::string> sorted_controller_names = motion_controller_names_;
  std::sort(sorted_controller_names.begin(), sorted_controller_names.end());
  const bool has_empty_name = std::any_of(
    motion_controller_names_.begin(), motion_controller_names_.end(),
    [](const std::string & name) {return name.empty();});
  const bool has_duplicate_name = std::adjacent_find(
    sorted_controller_names.begin(), sorted_controller_names.end()) !=
    sorted_controller_names.end();
  const bool default_is_registered = std::find(
    motion_controller_names_.begin(), motion_controller_names_.end(),
    default_motion_controller_) != motion_controller_names_.end();
  const bool rolling_is_registered = std::find(
    motion_controller_names_.begin(), motion_controller_names_.end(),
    rolling_motion_controller_) != motion_controller_names_.end();
  const auto valid_threshold_array = [](const std::vector<double> & values) {
      return values.size() == kAxisCount && std::all_of(
        values.begin(), values.end(),
        [](double value) {return std::isfinite(value) && value >= 0.0;});
    };

  if (
    batch_timeout_seconds_ <= 0.0 || disable_stage_timeout_seconds_ <= 0.0 ||
    inter_batch_delay_seconds_ < 0.0 || fault_reset_timeout_seconds_ <= 0.0 ||
    controller_switch_timeout_seconds_ <= 0.0 || service_timeout <= 0 ||
    motion_controller_names_.empty() || has_empty_name || has_duplicate_name ||
    default_motion_controller_.empty() || !default_is_registered ||
    rolling_motion_controller_.empty() || !rolling_is_registered ||
    rolling_motion_controller_ == default_motion_controller_ || maximum_sample_period_ms <= 0 ||
    source_state_max_age_ms <= 0 || stable_interval_count <= 0 ||
    mode_switch_timeout_ms <= 0 || !std::isfinite(mode_switch_feedback_age_limit_ms) ||
    mode_switch_feedback_age_limit_ms <= 0.0 ||
    !valid_threshold_array(stable_velocity_thresholds) ||
    !valid_threshold_array(takeover_tolerances))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid enable-manager timing or controller parameter");
    return controller_interface::CallbackReturn::ERROR;
  }
  service_result_timeout_ = std::chrono::milliseconds(service_timeout);
  mode_switch_maximum_sample_period_ns_ =
    static_cast<std::uint64_t>(maximum_sample_period_ms) * 1000000U;
  mode_switch_source_state_max_age_ns_ =
    static_cast<std::uint64_t>(source_state_max_age_ms) * 1000000U;
  mode_switch_stable_interval_count_ = static_cast<std::size_t>(stable_interval_count);
  mode_switch_timeout_ = std::chrono::milliseconds(mode_switch_timeout_ms);
  mode_switch_feedback_age_limit_ms_ = mode_switch_feedback_age_limit_ms;
  std::copy(
    stable_velocity_thresholds.begin(), stable_velocity_thresholds.end(),
    mode_switch_stable_velocity_thresholds_.begin());
  std::copy(
    takeover_tolerances.begin(), takeover_tolerances.end(),
    mode_switch_takeover_tolerances_.begin());

  enable_callback_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  disable_callback_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  reset_callback_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  mode_callback_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  worker_callback_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  enable_service_ = get_node()->create_service<rt_control_interfaces::srv::RtEnable>(
    "/rt/enable", std::bind(
      &EnableManagerController::handleEnable, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, enable_callback_group_);
  disable_service_ = get_node()->create_service<rt_control_interfaces::srv::RtEnable>(
    "/rt/disable", std::bind(
      &EnableManagerController::handleDisable, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, disable_callback_group_);
  reset_service_ = get_node()->create_service<rt_control_interfaces::srv::RtEnable>(
    "/rt/reset_fault", std::bind(
      &EnableManagerController::handleResetFault, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, reset_callback_group_);
  mode_service_ = get_node()->create_service<ModeService>(
    "/rt/joint_control/set_mode", std::bind(
      &EnableManagerController::handleSetMode, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, mode_callback_group_);
  rclcpp::SubscriptionOptions source_subscription_options;
  source_subscription_options.callback_group = worker_callback_group_;
  jtc_state_subscription_ =
    get_node()->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "/whole_body_jtc/controller_state", robot_interfaces_qos::state(),
    std::bind(&EnableManagerController::handleJtcState, this, std::placeholders::_1),
    source_subscription_options);
  rolling_state_subscription_ =
    get_node()->create_subscription<
    robot_rt_control_interfaces::msg::RollingJointControlState>(
    "/rt/rolling_joint_control/state", robot_interfaces_qos::rolling_state(),
    std::bind(&EnableManagerController::handleRollingState, this, std::placeholders::_1),
    source_subscription_options);
  list_client_ = get_node()->create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers", rmw_qos_profile_services_default,
    worker_callback_group_);
  switch_client_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller", rmw_qos_profile_services_default,
    worker_callback_group_);

  diagnostics_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", robot_interfaces_qos::diagnostic());
  worker_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&EnableManagerController::handleNonRtFaultStop, this), worker_callback_group_);
  diagnostics_timer_ = get_node()->create_wall_timer(
    std::chrono::seconds(1), std::bind(&EnableManagerController::publishDiagnostics, this),
    worker_callback_group_);

  active_.store(false, std::memory_order_release);
  phase_.store(Phase::kInactive, std::memory_order_release);
  owner_.store(Owner::kInternal, std::memory_order_release);
  current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
  mode_cache_.fill(ModeCacheEntry{});
  next_mode_cache_slot_ = 0U;
  {
    std::lock_guard<std::mutex> lock(command_snapshot_mutex_);
    jtc_command_snapshot_ = CommandSnapshot{};
    rolling_command_snapshot_ = CommandSnapshot{};
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EnableManagerController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (
    command_interfaces_.size() != kAxisCount ||
    state_interfaces_.size() != kAxisCount * 2U + 1U)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 14 control_word, 14 status_word, 14 position and feedback-age interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const std::string expected_status = std::string(kJointNames[axis]) + "/status_word";
    const std::string expected_position = std::string(kJointNames[axis]) + "/position";
    if (
      state_interfaces_[axis].get_name() != expected_status ||
      state_interfaces_[kAxisCount + axis].get_name() != expected_position)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unexpected state interface order");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (state_interfaces_[kAxisCount * 2U].get_name() !=
    "ethercat_domain/process_data_age_ms")
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing feedback-age state interface");
    return controller_interface::CallbackReturn::ERROR;
  }

  setAllControlWords(0x0000U);
  enable_request_.store(false, std::memory_order_release);
  disable_request_.store(false, std::memory_order_release);
  reset_request_.store(false, std::memory_order_release);
  enable_hardware_ready_.store(false, std::memory_order_release);
  jtc_activate_failed_request_.store(false, std::memory_order_release);
  emergency_jtc_deactivate_request_.store(false, std::memory_order_release);
  jtc_deactivation_required_.store(false, std::memory_order_release);
  restart_required_.store(false, std::memory_order_release);
  mode_callback_active_.store(false, std::memory_order_release);
  mode_abort_requested_.store(false, std::memory_order_release);
  switch_in_progress_.store(false, std::memory_order_release);
  current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
  mode_cache_.fill(ModeCacheEntry{});
  next_mode_cache_slot_ = 0U;
  {
    std::lock_guard<std::mutex> lock(command_snapshot_mutex_);
    jtc_command_snapshot_ = CommandSnapshot{};
    rolling_command_snapshot_ = CommandSnapshot{};
  }
  actual_sample_version_.store(0U, std::memory_order_relaxed);
  actual_sample_sequence_.store(0U, std::memory_order_relaxed);
  actual_sample_time_ns_.store(0U, std::memory_order_relaxed);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    actual_position_bits_[axis].store(
      encodeDouble(state_interfaces_[kAxisCount + axis].get_value()),
      std::memory_order_relaxed);
  }
  actual_feedback_age_bits_.store(
    encodeDouble(state_interfaces_[kAxisCount * 2U].get_value()),
    std::memory_order_relaxed);
  clearFailure();
  current_batch_ = 0U;
  downward_stage_ = 0U;
  stage_deadline_ns_ = 0;
  enable_preempt_requested_ = false;
  jtc_deactivate_failed_.store(false, std::memory_order_release);
  primary_failure_stage_ = Stage::kSuccess;
  primary_failed_batch_ = -1;
  primary_failed_joint_ = -1;
  primary_failed_status_word_ = 0U;
  owner_.store(Owner::kInternal, std::memory_order_release);
  phase_.store(Phase::kStartupSanitizing, std::memory_order_release);
  active_.store(true, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EnableManagerController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  mode_abort_requested_.store(true, std::memory_order_release);
  setAllControlWords(0x0000U);
  phase_.store(Phase::kInactive, std::memory_order_release);
  owner_.store(Owner::kInternal, std::memory_order_release);
  current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type EnableManagerController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!active_.load(std::memory_order_acquire)) {
    return controller_interface::return_type::OK;
  }

  refreshStatusWords();
  actual_sample_version_.fetch_add(1U, std::memory_order_acq_rel);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    actual_position_bits_[axis].store(
      encodeDouble(state_interfaces_[kAxisCount + axis].get_value()),
      std::memory_order_relaxed);
  }
  actual_feedback_age_bits_.store(
    encodeDouble(state_interfaces_[kAxisCount * 2U].get_value()),
    std::memory_order_relaxed);
  actual_sample_time_ns_.store(steadyNowNs(), std::memory_order_relaxed);
  actual_sample_sequence_.fetch_add(1U, std::memory_order_relaxed);
  actual_sample_version_.fetch_add(1U, std::memory_order_release);
  const std::int64_t now_ns = time.nanoseconds();
  Phase phase = phase_.load(std::memory_order_acquire);

  if (phase == Phase::kEnabled || phase == Phase::kJtcActivating ||
    phase == Phase::kJtcDeactivating)
  {
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      const DriveState state = decodeState(status_words_[axis]);
      if (isFaultState(state)) {
        startEmergency(
          Stage::kFaultDetected, static_cast<std::int8_t>(axis), status_words_[axis], now_ns);
        phase = Phase::kEmergencyQuickStop;
        break;
      }
      if (phase == Phase::kEnabled && state != DriveState::kOperationEnabled) {
        startEmergency(
          Stage::kUnexpectedDriveState, static_cast<std::int8_t>(axis), status_words_[axis],
          now_ns);
        phase = Phase::kEmergencyQuickStop;
        break;
      }
    }
  }

  switch (phase) {
    case Phase::kStartupSanitizing:
      if (stage_deadline_ns_ == 0) {
        startDownward(Phase::kStartupSanitizing, now_ns);
      }
      updateDownward(now_ns);
      break;
    case Phase::kIdle:
      setAllControlWords(0x0000U);
      if (enable_request_.exchange(false, std::memory_order_acq_rel)) {
        current_batch_ = 0U;
        enable_preempt_requested_ = false;
        primary_failure_stage_ = Stage::kSuccess;
        primary_failed_batch_ = -1;
        primary_failed_joint_ = -1;
        primary_failed_status_word_ = 0U;
        enable_hardware_ready_.store(false, std::memory_order_release);
        stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(batch_timeout_seconds_ * 1e9);
        phase_.store(Phase::kEnabling, std::memory_order_release);
      } else if (reset_request_.exchange(false, std::memory_order_acq_rel)) {
        reset_targets_.fill(false);
        bool has_fault = false;
        for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
          reset_targets_[axis] = isFaultState(decodeState(status_words_[axis]));
          has_fault = has_fault || reset_targets_[axis];
        }
        if (!has_fault) {
          clearFailure();
          publishResult(reset_result_, true, Stage::kAlreadyClear);
          owner_.store(Owner::kNone, std::memory_order_release);
        } else {
          setAllControlWords(0x0000U);
          phase_.store(Phase::kResetLow, std::memory_order_release);
        }
      } else if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
        clearFailure();
        publishResult(disable_result_, true, Stage::kAlreadyDisabled);
        owner_.store(Owner::kNone, std::memory_order_release);
      }
      break;
    case Phase::kEnabling:
    case Phase::kInterBatchDelay:
      updateEnable(now_ns);
      break;
    case Phase::kJtcActivating:
      if (jtc_activate_failed_request_.exchange(false, std::memory_order_acq_rel)) {
        primary_failure_stage_ = Stage::kJtcActivateFailed;
        primary_failed_batch_ = -1;
        primary_failed_joint_ = -1;
        primary_failed_status_word_ = 0U;
        recordFailure(Stage::kJtcActivateFailed, -1, -1, 0U);
        startDownward(Phase::kRollback, now_ns);
        break;
      }
      setAllControlWords(0x000FU);
      break;
    case Phase::kJtcDeactivating:
    case Phase::kEnabled:
      setAllControlWords(0x000FU);
      if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
        startDownward(Phase::kDisabling, now_ns);
      }
      break;
    case Phase::kDisabling:
    case Phase::kRollback:
    case Phase::kResetDisabling:
    case Phase::kEmergencyDisable:
      if (stage_deadline_ns_ == 0) {
        startDownward(phase, now_ns);
      }
      updateDownward(now_ns);
      break;
    case Phase::kResetLow:
    case Phase::kResetHigh:
      updateReset(now_ns);
      break;
    case Phase::kEmergencyQuickStop:
      updateEmergencyQuickStop(now_ns);
      break;
    case Phase::kFailed:
      setAllControlWords(0x0000U);
      if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
        startDownward(Phase::kDisabling, now_ns);
      } else if (reset_request_.exchange(false, std::memory_order_acq_rel)) {
        reset_targets_.fill(false);
        bool has_fault = false;
        for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
          reset_targets_[axis] = isFaultState(decodeState(status_words_[axis]));
          has_fault = has_fault || reset_targets_[axis];
        }
        if (!has_fault) {
          clearFailure();
          publishResult(reset_result_, true, Stage::kAlreadyClear);
          phase_.store(Phase::kIdle, std::memory_order_release);
          owner_.store(Owner::kNone, std::memory_order_release);
        } else {
          phase_.store(Phase::kResetLow, std::memory_order_release);
        }
      }
      break;
    case Phase::kInactive:
      break;
  }

  return controller_interface::return_type::OK;
}

void EnableManagerController::handleEnable(
  const std::shared_ptr<rt_control_interfaces::srv::RtEnable::Request>,
  std::shared_ptr<rt_control_interfaces::srv::RtEnable::Response> response)
{
  bool expected_callback = false;
  if (!enable_callback_active_.compare_exchange_strong(expected_callback, true)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    return;
  }
  const auto release_callback = [this]() {
      enable_callback_active_.store(false, std::memory_order_release);
    };

  if (!active_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerInactive);
    release_callback();
    return;
  }
  if (restart_required_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kRestartRequired);
    release_callback();
    return;
  }
  if (phase_.load(std::memory_order_acquire) == Phase::kEnabled) {
    fillImmediateResponse(*response, true, Stage::kAlreadyEnabled);
    release_callback();
    return;
  }

  Owner expected_owner = Owner::kNone;
  if (
    phase_.load(std::memory_order_acquire) != Phase::kIdle ||
    !owner_.compare_exchange_strong(expected_owner, Owner::kEnable))
  {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }

  enable_result_.ready.store(false, std::memory_order_release);
  enable_hardware_ready_.store(false, std::memory_order_release);
  enable_request_.store(true, std::memory_order_release);
  const auto deadline = std::chrono::steady_clock::now() + service_result_timeout_;
  while (
    active_.load(std::memory_order_acquire) &&
    !enable_hardware_ready_.load(std::memory_order_acquire) &&
    !enable_result_.ready.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  if (enable_result_.ready.load(std::memory_order_acquire)) {
    fillResponse(enable_result_, *response);
    release_callback();
    return;
  }
  if (!enable_hardware_ready_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
    release_callback();
    return;
  }

  jtc_deactivation_required_.store(true, std::memory_order_release);
  const SwitchResult activation_result = switchJtc(true);
  if (activation_result != SwitchResult::kSuccess) {
    if (activation_result == SwitchResult::kFailed) {
      jtc_deactivation_required_.store(false, std::memory_order_release);
    }
    if (activation_result == SwitchResult::kAmbiguous) {
      restart_required_.store(true, std::memory_order_release);
      emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
    }
    if (phase_.load(std::memory_order_acquire) == Phase::kJtcActivating) {
      jtc_activate_failed_request_.store(true, std::memory_order_release);
    }
    if (waitForResult(enable_result_)) {
      fillResponse(enable_result_, *response);
    } else {
      fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
    }
    release_callback();
    return;
  }

  if (phase_.load(std::memory_order_acquire) == Phase::kJtcActivating) {
    clearFailure();
    current_control_mode_.store(ControlMode::kFjtReady, std::memory_order_release);
    phase_.store(Phase::kEnabled, std::memory_order_release);
    publishResult(enable_result_, true, Stage::kSuccess);
    owner_.store(Owner::kNone, std::memory_order_release);
  }
  if (waitForResult(enable_result_)) {
    fillResponse(enable_result_, *response);
  } else {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
  }
  release_callback();
}

void EnableManagerController::handleDisable(
  const std::shared_ptr<rt_control_interfaces::srv::RtEnable::Request>,
  std::shared_ptr<rt_control_interfaces::srv::RtEnable::Response> response)
{
  bool expected_callback = false;
  if (!disable_callback_active_.compare_exchange_strong(expected_callback, true)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    return;
  }
  const auto release_callback = [this]() {
      disable_callback_active_.store(false, std::memory_order_release);
    };
  if (!active_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerInactive);
    release_callback();
    return;
  }

  Phase phase = phase_.load(std::memory_order_acquire);
  if (phase == Phase::kIdle) {
    fillImmediateResponse(*response, true, Stage::kAlreadyDisabled);
    release_callback();
    return;
  }
  if (
    phase == Phase::kStartupSanitizing || phase == Phase::kEmergencyQuickStop ||
    phase == Phase::kEmergencyDisable || phase == Phase::kJtcActivating ||
    phase == Phase::kDisabling || phase == Phase::kRollback)
  {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }

  Owner current_owner = owner_.load(std::memory_order_acquire);
  if (current_owner == Owner::kMode) {
    mode_abort_requested_.store(true, std::memory_order_release);
    const auto mode_deadline = std::chrono::steady_clock::now() + mode_switch_timeout_ * 5;
    while (
      active_.load(std::memory_order_acquire) &&
      owner_.load(std::memory_order_acquire) == Owner::kMode &&
      std::chrono::steady_clock::now() < mode_deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    current_owner = owner_.load(std::memory_order_acquire);
    phase = phase_.load(std::memory_order_acquire);
    if (current_owner == Owner::kMode) {
      restart_required_.store(true, std::memory_order_release);
      jtc_deactivation_required_.store(true, std::memory_order_release);
      emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
      disable_request_.store(true, std::memory_order_release);
      fillImmediateResponse(*response, false, Stage::kRestartRequired);
      release_callback();
      return;
    }
    if (phase != Phase::kEnabled) {
      fillImmediateResponse(*response, false, Stage::kOperationInProgress);
      release_callback();
      return;
    }
  }
  const bool preempt_enable =
    current_owner == Owner::kEnable &&
    (phase == Phase::kEnabling || phase == Phase::kInterBatchDelay);
  const bool preempt_reset =
    current_owner == Owner::kReset &&
    (phase == Phase::kResetLow || phase == Phase::kResetHigh);
  if (!preempt_enable && !preempt_reset) {
    Owner expected_owner = Owner::kNone;
    if (!owner_.compare_exchange_strong(expected_owner, Owner::kDisable)) {
      fillImmediateResponse(*response, false, Stage::kOperationInProgress);
      release_callback();
      return;
    }
  }

  disable_result_.ready.store(false, std::memory_order_release);
  jtc_deactivate_failed_.store(false, std::memory_order_release);
  if (phase == Phase::kEnabled) {
    phase_.store(Phase::kJtcDeactivating, std::memory_order_release);
    if (switchJtc(false) != SwitchResult::kSuccess) {
      jtc_deactivate_failed_.store(true, std::memory_order_release);
      restart_required_.store(true, std::memory_order_release);
      recordFailure(Stage::kJtcDeactivateFailed, -1, -1, 0U);
    } else {
      jtc_deactivation_required_.store(false, std::memory_order_release);
    }
  }
  disable_request_.store(true, std::memory_order_release);
  if (waitForResult(disable_result_)) {
    fillResponse(disable_result_, *response);
  } else {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
  }
  release_callback();
}

void EnableManagerController::handleResetFault(
  const std::shared_ptr<rt_control_interfaces::srv::RtEnable::Request>,
  std::shared_ptr<rt_control_interfaces::srv::RtEnable::Response> response)
{
  bool expected_callback = false;
  if (!reset_callback_active_.compare_exchange_strong(expected_callback, true)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    return;
  }
  const auto release_callback = [this]() {
      reset_callback_active_.store(false, std::memory_order_release);
    };
  if (!active_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerInactive);
    release_callback();
    return;
  }
  if (restart_required_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kRestartRequired);
    release_callback();
    return;
  }

  const Phase phase = phase_.load(std::memory_order_acquire);
  if (phase != Phase::kIdle && phase != Phase::kFailed) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }
  Owner expected_owner = Owner::kNone;
  if (!owner_.compare_exchange_strong(expected_owner, Owner::kReset)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }

  reset_result_.ready.store(false, std::memory_order_release);
  reset_request_.store(true, std::memory_order_release);
  if (waitForResult(reset_result_)) {
    fillResponse(reset_result_, *response);
  } else {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
  }
  release_callback();
}

void EnableManagerController::handleJtcState(
  const control_msgs::msg::JointTrajectoryControllerState::SharedPtr message)
{
  if (message->joint_names.size() != kAxisCount ||
    message->output.positions.size() != kAxisCount)
  {
    return;
  }
  CommandSnapshot snapshot;
  snapshot.valid = true;
  snapshot.control_mode = static_cast<std::uint8_t>(ControlMode::kFjtReady);
  snapshot.received_steady_ns = steadyNowNs();
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (
      message->joint_names[axis] != kJointNames[axis] ||
      !std::isfinite(message->output.positions[axis]))
    {
      return;
    }
    snapshot.positions[axis] = message->output.positions[axis];
  }
  std::lock_guard<std::mutex> lock(command_snapshot_mutex_);
  jtc_command_snapshot_ = snapshot;
}

void EnableManagerController::handleRollingState(
  const robot_rt_control_interfaces::msg::RollingJointControlState::SharedPtr message)
{
  CommandSnapshot snapshot;
  snapshot.valid = true;
  snapshot.control_mode = message->control_mode.value;
  snapshot.received_steady_ns = steadyNowNs();
  snapshot.controller_boot_id = message->controller_boot_id.uuid;
  snapshot.has_session = message->has_session;
  snapshot.session_state = message->session_state.value;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (!std::isfinite(message->desired_positions[axis])) {
      return;
    }
    snapshot.positions[axis] = message->desired_positions[axis];
  }
  std::lock_guard<std::mutex> lock(command_snapshot_mutex_);
  rolling_command_snapshot_ = snapshot;
}

bool EnableManagerController::sameModeRequest(
  const ModeService::Request & lhs, const ModeService::Request & rhs) noexcept
{
  return lhs.protocol_major == rhs.protocol_major &&
         lhs.protocol_minor == rhs.protocol_minor &&
         lhs.client_instance_id.uuid == rhs.client_instance_id.uuid &&
         lhs.request_id.uuid == rhs.request_id.uuid &&
         lhs.expected_mode.value == rhs.expected_mode.value &&
         lhs.target_mode.value == rhs.target_mode.value;
}

EnableManagerController::ModeCacheEntry * EnableManagerController::findModeCache(
  const std::array<std::uint8_t, 16U> & client_id,
  const std::array<std::uint8_t, 16U> & request_id) noexcept
{
  const auto entry = std::find_if(
    mode_cache_.begin(), mode_cache_.end(),
    [&client_id, &request_id](const ModeCacheEntry & candidate) {
      return candidate.valid && candidate.request.client_instance_id.uuid == client_id &&
      candidate.request.request_id.uuid == request_id;
    });
  return entry == mode_cache_.end() ? nullptr : &*entry;
}

EnableManagerController::ModeCacheEntry & EnableManagerController::allocateModeCache() noexcept
{
  ModeCacheEntry & entry = mode_cache_[next_mode_cache_slot_];
  next_mode_cache_slot_ = (next_mode_cache_slot_ + 1U) % kModeCacheCapacity;
  entry = ModeCacheEntry{};
  return entry;
}

bool EnableManagerController::readActualSample(ActualSample & sample) const noexcept
{
  constexpr std::size_t kMaximumReadAttempts = 5U;
  for (std::size_t attempt = 0U; attempt < kMaximumReadAttempts; ++attempt) {
    const std::uint64_t version_before =
      actual_sample_version_.load(std::memory_order_acquire);
    if ((version_before & 1U) != 0U) {
      continue;
    }
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      sample.positions[axis] = decodeDouble(
        actual_position_bits_[axis].load(std::memory_order_relaxed));
    }
    sample.feedback_age_ms = decodeDouble(
      actual_feedback_age_bits_.load(std::memory_order_relaxed));
    sample.steady_time_ns = actual_sample_time_ns_.load(std::memory_order_relaxed);
    sample.sequence = actual_sample_sequence_.load(std::memory_order_relaxed);
    const std::uint64_t version_after =
      actual_sample_version_.load(std::memory_order_acquire);
    if (version_before == version_after && (version_after & 1U) == 0U) {
      return sample.sequence != 0U && sample.steady_time_ns != 0U;
    }
  }
  return false;
}

bool EnableManagerController::readCommandSnapshot(
  ControlMode mode, CommandSnapshot & snapshot) const
{
  std::lock_guard<std::mutex> lock(command_snapshot_mutex_);
  if (mode == ControlMode::kFjtReady) {
    snapshot = jtc_command_snapshot_;
  } else if (mode == ControlMode::kRollingReady) {
    snapshot = rolling_command_snapshot_;
  } else {
    return false;
  }
  return snapshot.valid && snapshot.control_mode == static_cast<std::uint8_t>(mode);
}

std::uint8_t EnableManagerController::validateModeSource(
  ControlMode source, CommandSnapshot & snapshot)
{
  using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
  if (!readCommandSnapshot(source, snapshot)) {
    return ServiceResult::SOURCE_STATE_STALE;
  }
  std::uint64_t now_ns = steadyNowNs();
  if (
    now_ns < snapshot.received_steady_ns ||
    now_ns - snapshot.received_steady_ns > mode_switch_source_state_max_age_ns_)
  {
    return ServiceResult::SOURCE_STATE_STALE;
  }
  if (source == ControlMode::kRollingReady && snapshot.has_session) {
    return ServiceResult::SESSION_EXISTS;
  }

  ActualSample previous;
  if (!readActualSample(previous)) {
    return ServiceResult::NOT_READY;
  }
  if (
    !std::isfinite(previous.feedback_age_ms) || previous.feedback_age_ms < 0.0 ||
    previous.feedback_age_ms > mode_switch_feedback_age_limit_ms_)
  {
    return ServiceResult::FEEDBACK_STALE;
  }

  std::size_t stable_intervals = 0U;
  const auto deadline = std::chrono::steady_clock::now() + mode_switch_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    if (
      mode_abort_requested_.load(std::memory_order_acquire) ||
      owner_.load(std::memory_order_acquire) != Owner::kMode ||
      phase_.load(std::memory_order_acquire) != Phase::kEnabled)
    {
      return ServiceResult::NOT_READY;
    }
    ActualSample current;
    if (!readActualSample(current) || current.sequence == previous.sequence) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (
      !std::isfinite(current.feedback_age_ms) || current.feedback_age_ms < 0.0 ||
      current.feedback_age_ms > mode_switch_feedback_age_limit_ms_)
    {
      return ServiceResult::FEEDBACK_STALE;
    }
    stable_intervals = stableActualInterval(previous, current) ?
      stable_intervals + 1U : 0U;
    previous = current;
    if (stable_intervals < mode_switch_stable_interval_count_) {
      continue;
    }

    CommandSnapshot fresh_source;
    if (!readCommandSnapshot(source, fresh_source)) {
      return ServiceResult::SOURCE_STATE_STALE;
    }
    now_ns = steadyNowNs();
    if (
      now_ns < fresh_source.received_steady_ns ||
      now_ns - fresh_source.received_steady_ns > mode_switch_source_state_max_age_ns_)
    {
      return ServiceResult::SOURCE_STATE_STALE;
    }
    if (source == ControlMode::kRollingReady && fresh_source.has_session) {
      return ServiceResult::SESSION_EXISTS;
    }
    if (!sourceWithinTakeoverTolerance(fresh_source.positions, current.positions)) {
      return ServiceResult::TAKEOVER_MISMATCH;
    }
    snapshot = fresh_source;
    return ServiceResult::NONE;
  }
  return ServiceResult::SOURCE_MOVING;
}

bool EnableManagerController::queryControllerStates(
  controller_manager_msgs::srv::ListControllers::Response & states,
  std::chrono::milliseconds timeout)
{
  if (!list_client_->wait_for_service(std::chrono::milliseconds(0))) {
    return false;
  }
  auto request =
    std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = list_client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    return false;
  }
  const auto response = future.get();
  if (response == nullptr) {
    return false;
  }
  states = *response;
  return true;
}

const char * EnableManagerController::controllerNameForMode(
  ControlMode mode, const std::string & default_controller,
  const std::string & rolling_controller) noexcept
{
  if (mode == ControlMode::kFjtReady) {
    return default_controller.c_str();
  }
  if (mode == ControlMode::kRollingReady) {
    return rolling_controller.c_str();
  }
  return nullptr;
}

bool EnableManagerController::waitForRollingActivationEvidence(
  std::uint64_t not_before_ns, CommandSnapshot & snapshot)
{
  const auto deadline = std::chrono::steady_clock::now() + mode_switch_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    if (
      readCommandSnapshot(ControlMode::kRollingReady, snapshot) &&
      snapshot.received_steady_ns >= not_before_ns &&
      !isZeroIdentifier(snapshot.controller_boot_id))
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

void EnableManagerController::convergeAfterUnsafeModeSwitch(bool require_restart)
{
  restart_required_.store(require_restart, std::memory_order_release);
  current_control_mode_.store(
    require_restart ? ControlMode::kRestartRequired : ControlMode::kDisabled,
    std::memory_order_release);
  disable_result_.ready.store(false, std::memory_order_release);
  jtc_deactivation_required_.store(true, std::memory_order_release);
  owner_.store(Owner::kDisable, std::memory_order_release);
  if (require_restart) {
    emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
  }
  disable_request_.store(true, std::memory_order_release);
}

bool EnableManagerController::releaseModeOwnership() noexcept
{
  Owner expected_owner = Owner::kMode;
  return owner_.compare_exchange_strong(
    expected_owner, Owner::kNone, std::memory_order_acq_rel);
}

std::uint8_t EnableManagerController::executeModeSwitch(
  ControlMode source, ControlMode target, ModeService::Response & response,
  std::uint64_t switch_started_ns)
{
  using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
  if (mode_abort_requested_.load(std::memory_order_acquire)) {
    return ServiceResult::NOT_READY;
  }
  bool switch_expected = false;
  if (!switch_in_progress_.compare_exchange_strong(
      switch_expected, true, std::memory_order_acq_rel))
  {
    return ServiceResult::NOT_READY;
  }
  const auto release_switch = [this]() {
      switch_in_progress_.store(false, std::memory_order_release);
    };
  const auto safety_preempted = [this]() {
      return owner_.load(std::memory_order_acquire) != Owner::kMode ||
             phase_.load(std::memory_order_acquire) != Phase::kEnabled;
    };
  if (mode_abort_requested_.load(std::memory_order_acquire)) {
    release_switch();
    return ServiceResult::NOT_READY;
  }
  controller_manager_msgs::srv::ListControllers::Response before;
  if (!queryControllerStates(before, mode_switch_timeout_)) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return ServiceResult::RESTART_REQUIRED;
  }
  if (safety_preempted()) {
    restart_required_.store(true, std::memory_order_release);
    current_control_mode_.store(ControlMode::kRestartRequired, std::memory_order_release);
    release_switch();
    return ServiceResult::RESTART_REQUIRED;
  }
  if (detectControlMode(before) != source) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return ServiceResult::RESTART_REQUIRED;
  }
  if (mode_abort_requested_.load(std::memory_order_acquire)) {
    release_switch();
    return ServiceResult::NOT_READY;
  }
  if (!switch_client_->wait_for_service(std::chrono::milliseconds(0))) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return ServiceResult::RESTART_REQUIRED;
  }

  const char * source_controller = controllerNameForMode(
    source, default_motion_controller_, rolling_motion_controller_);
  const char * target_controller = controllerNameForMode(
    target, default_motion_controller_, rolling_motion_controller_);
  if (source_controller == nullptr || target_controller == nullptr) {
    release_switch();
    return ServiceResult::WRONG_MODE;
  }
  auto request =
    std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers.emplace_back(source_controller);
  request->activate_controllers.emplace_back(target_controller);
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  const std::int64_t timeout_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(mode_switch_timeout_).count();
  request->timeout.sec = static_cast<std::int32_t>(timeout_ns / 1000000000LL);
  request->timeout.nanosec = static_cast<std::uint32_t>(timeout_ns % 1000000000LL);
  auto future = switch_client_->async_send_request(request);
  const bool response_ready =
    future.wait_for(mode_switch_timeout_) == std::future_status::ready;
  const auto switch_response = response_ready ? future.get() : nullptr;
  const bool switch_ok = switch_response != nullptr && switch_response->ok;

  controller_manager_msgs::srv::ListControllers::Response after;
  if (!queryControllerStates(after, mode_switch_timeout_)) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return ServiceResult::RESTART_REQUIRED;
  }
  const auto controller_has_state = [&after](const char * name, const char * state) {
      const auto controller = std::find_if(
        after.controller.begin(), after.controller.end(),
        [name](const auto & candidate) {return candidate.name == name;});
      return controller != after.controller.end() && controller->state == state;
    };
  const ModeSwitchState observed = classifyModeSwitchState(after, source, target);
  response.source_controller_deactivated = controller_has_state(source_controller, "inactive");
  response.target_controller_activated = controller_has_state(target_controller, "active");
  if (safety_preempted()) {
    restart_required_.store(true, std::memory_order_release);
    current_control_mode_.store(ControlMode::kRestartRequired, std::memory_order_release);
    release_switch();
    return ServiceResult::RESTART_REQUIRED;
  }
  if (!response_ready) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return ServiceResult::RESTART_REQUIRED;
  }
  if (switch_ok && observed == ModeSwitchState::kTargetActive) {
    if (target == ControlMode::kRollingReady) {
      CommandSnapshot rolling_snapshot;
      if (!waitForRollingActivationEvidence(switch_started_ns, rolling_snapshot)) {
        release_switch();
        convergeAfterUnsafeModeSwitch(true);
        return ServiceResult::RESTART_REQUIRED;
      }
      response.controller_boot_id.uuid = rolling_snapshot.controller_boot_id;
    }
    current_control_mode_.store(target, std::memory_order_release);
    release_switch();
    return ServiceResult::NONE;
  }
  if (!switch_ok && observed == ModeSwitchState::kSourcePreserved) {
    release_switch();
    return ServiceResult::SWITCH_REJECTED;
  }
  if (observed == ModeSwitchState::kAllInactive) {
    release_switch();
    convergeAfterUnsafeModeSwitch(false);
    return ServiceResult::SWITCH_REJECTED;
  }
  release_switch();
  convergeAfterUnsafeModeSwitch(true);
  return ServiceResult::RESTART_REQUIRED;
}

void EnableManagerController::setModeResponse(
  ModeService::Response & response, const ModeService::Request & request,
  std::uint8_t result, ControlMode mode, bool source_deactivated,
  bool target_activated, bool require_restart)
{
  using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
  response.accepted = result == ServiceResult::NONE;
  response.result.value = result;
  response.request_id = request.request_id;
  response.mode.value = static_cast<std::uint8_t>(mode);
  response.source_controller_deactivated = source_deactivated;
  response.target_controller_activated = target_activated;
  response.restart_required = require_restart;
  populateModeError(response.error, result);
}

void EnableManagerController::handleSetMode(
  const std::shared_ptr<ModeService::Request> request,
  std::shared_ptr<ModeService::Response> response)
{
  using ModeMessage = robot_rt_control_interfaces::msg::JointControlMode;
  using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
  *response = ModeService::Response{};
  response->request_id = request->request_id;

  bool expected_callback = false;
  if (!mode_callback_active_.compare_exchange_strong(expected_callback, true)) {
    setModeResponse(
      *response, *request, ServiceResult::NOT_READY,
      current_control_mode_.load(std::memory_order_acquire), false, false,
      restart_required_.load(std::memory_order_acquire));
    return;
  }
  const auto release_callback = [this]() {
      mode_callback_active_.store(false, std::memory_order_release);
    };
  const auto finish = [this, &request, &response, &release_callback](
    std::uint8_t result, ControlMode mode, bool source_deactivated,
    bool target_activated, bool require_restart, bool cache) {
      setModeResponse(
        *response, *request, result, mode, source_deactivated, target_activated,
        require_restart);
      if (cache) {
        ModeCacheEntry & entry = allocateModeCache();
        entry.valid = true;
        entry.request = *request;
        entry.response = *response;
      }
      release_callback();
    };

  const bool identifiers_valid =
    !isZeroIdentifier(request->client_instance_id.uuid) &&
    !isZeroIdentifier(request->request_id.uuid);
  if (identifiers_valid) {
    ModeCacheEntry * cached = findModeCache(
      request->client_instance_id.uuid, request->request_id.uuid);
    if (cached != nullptr) {
      if (sameModeRequest(cached->request, *request)) {
        *response = cached->response;
      } else {
        setModeResponse(
          *response, *request, ServiceResult::WRONG_REQUEST,
          current_control_mode_.load(std::memory_order_acquire), false, false,
          restart_required_.load(std::memory_order_acquire));
      }
      release_callback();
      return;
    }
  }
  if (!identifiers_valid) {
    finish(
      ServiceResult::WRONG_REQUEST,
      current_control_mode_.load(std::memory_order_acquire), false, false,
      restart_required_.load(std::memory_order_acquire), false);
    return;
  }
  if (request->protocol_major != 1U || request->protocol_minor != 0U) {
    finish(
      ServiceResult::WRONG_PROTOCOL,
      current_control_mode_.load(std::memory_order_acquire), false, false,
      restart_required_.load(std::memory_order_acquire), true);
    return;
  }
  if (!active_.load(std::memory_order_acquire)) {
    finish(
      ServiceResult::NOT_ENABLED, ControlMode::kDisabled, false, false, false, true);
    return;
  }
  if (restart_required_.load(std::memory_order_acquire)) {
    finish(
      ServiceResult::RESTART_REQUIRED, ControlMode::kRestartRequired, false, false,
      true, true);
    return;
  }
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (phase != Phase::kEnabled) {
    finish(
      phase == Phase::kIdle ? ServiceResult::NOT_ENABLED : ServiceResult::NOT_READY,
      current_control_mode_.load(std::memory_order_acquire), false, false, false,
      true);
    return;
  }
  const ControlMode expected_mode = static_cast<ControlMode>(request->expected_mode.value);
  const ControlMode target_mode = static_cast<ControlMode>(request->target_mode.value);
  const ControlMode current_mode = current_control_mode_.load(std::memory_order_acquire);
  const bool valid_expected =
    request->expected_mode.value == ModeMessage::FJT_READY ||
    request->expected_mode.value == ModeMessage::ROLLING_READY;
  const bool valid_target =
    request->target_mode.value == ModeMessage::FJT_READY ||
    request->target_mode.value == ModeMessage::ROLLING_READY;
  if (!valid_expected || !valid_target || expected_mode != current_mode) {
    finish(
      ServiceResult::WRONG_MODE, current_mode, false, false, false, true);
    return;
  }
  if (
    owner_.load(std::memory_order_acquire) != Owner::kNone)
  {
    finish(ServiceResult::NOT_READY, current_mode, false, false, false, true);
    return;
  }
  if (target_mode == current_mode) {
    CommandSnapshot current_snapshot;
    if (current_mode == ControlMode::kRollingReady &&
      readCommandSnapshot(current_mode, current_snapshot))
    {
      response->controller_boot_id.uuid = current_snapshot.controller_boot_id;
    }
    finish(ServiceResult::NONE, current_mode, false, true, false, true);
    return;
  }

  Owner expected_owner = Owner::kNone;
  mode_abort_requested_.store(false, std::memory_order_release);
  if (!owner_.compare_exchange_strong(expected_owner, Owner::kMode)) {
    finish(ServiceResult::NOT_READY, current_mode, false, false, false, true);
    return;
  }
  CommandSnapshot source_snapshot;
  const std::uint8_t admission_result = validateModeSource(current_mode, source_snapshot);
  if (admission_result != ServiceResult::NONE) {
    (void)releaseModeOwnership();
    finish(admission_result, current_mode, false, false, false, true);
    return;
  }
  if (current_mode == ControlMode::kRollingReady) {
    response->controller_boot_id.uuid = source_snapshot.controller_boot_id;
  }
  const std::uint64_t switch_started_ns = steadyNowNs();
  const std::uint8_t switch_result = executeModeSwitch(
    current_mode, target_mode, *response, switch_started_ns);
  (void)releaseModeOwnership();
  const ControlMode resulting_mode = current_control_mode_.load(std::memory_order_acquire);
  finish(
    switch_result, resulting_mode, response->source_controller_deactivated,
    response->target_controller_activated,
    resulting_mode == ControlMode::kRestartRequired, true);
}

bool EnableManagerController::waitForResult(ResultSlot & slot)
{
  const auto deadline = std::chrono::steady_clock::now() + service_result_timeout_;
  while (
    active_.load(std::memory_order_acquire) &&
    !slot.ready.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return slot.ready.load(std::memory_order_acquire);
}

void EnableManagerController::fillResponse(
  const ResultSlot & slot, rt_control_interfaces::srv::RtEnable::Response & response) const
{
  response.ok = slot.ok.load(std::memory_order_acquire);
  response.failed_batch = slot.failed_batch.load(std::memory_order_acquire);
  const std::int8_t failed_joint = slot.failed_joint.load(std::memory_order_acquire);
  response.failed_joint = failed_joint >= 0 ? kJointNames[failed_joint] : "";
  response.status_word = slot.status_word.load(std::memory_order_acquire);
  response.stage = stageName(slot.stage.load(std::memory_order_acquire));
}

void EnableManagerController::fillImmediateResponse(
  rt_control_interfaces::srv::RtEnable::Response & response, bool ok, Stage stage) const
{
  response.ok = ok;
  response.failed_batch = -1;
  response.failed_joint.clear();
  response.status_word = 0U;
  response.stage = stageName(stage);
}

EnableManagerController::SwitchResult
EnableManagerController::buildMotionControllerSwitchRequest(
  const controller_manager_msgs::srv::ListControllers::Response & states,
  bool activate_default,
  controller_manager_msgs::srv::SwitchController::Request & request) const
{
  request.activate_controllers.clear();
  request.deactivate_controllers.clear();
  for (const std::string & registered_name : motion_controller_names_) {
    const auto controller = std::find_if(
      states.controller.begin(), states.controller.end(),
      [&registered_name](const auto & candidate) {
        return candidate.name == registered_name;
      });
    if (controller == states.controller.end()) {
      return SwitchResult::kFailed;
    }
    const bool is_active = controller->state == "active";
    if (registered_name == default_motion_controller_ && activate_default) {
      if (!is_active) {
        if (controller->state != "inactive") {
          return SwitchResult::kFailed;
        }
        request.activate_controllers.push_back(registered_name);
      }
    } else if (is_active) {
      request.deactivate_controllers.push_back(registered_name);
    }
  }
  return SwitchResult::kSuccess;
}

bool EnableManagerController::registeredMotionControllerStateMatches(
  const controller_manager_msgs::srv::ListControllers::Response & states,
  bool default_active) const
{
  for (const std::string & registered_name : motion_controller_names_) {
    const auto controller = std::find_if(
      states.controller.begin(), states.controller.end(),
      [&registered_name](const auto & candidate) {
        return candidate.name == registered_name;
      });
    if (controller == states.controller.end()) {
      return false;
    }
    const std::string expected_state =
      default_active && registered_name == default_motion_controller_ ?
      "active" : "inactive";
    if (controller->state != expected_state) {
      return false;
    }
  }
  return true;
}

EnableManagerController::ControlMode EnableManagerController::detectControlMode(
  const controller_manager_msgs::srv::ListControllers::Response & states) const
{
  const auto state_for = [&states](const std::string & name) -> const std::string * {
      const auto controller = std::find_if(
        states.controller.begin(), states.controller.end(),
        [&name](const auto & candidate) {return candidate.name == name;});
      return controller == states.controller.end() ? nullptr : &controller->state;
    };
  const std::string * fjt_state = state_for(default_motion_controller_);
  const std::string * rolling_state = state_for(rolling_motion_controller_);
  if (fjt_state == nullptr || rolling_state == nullptr) {
    return ControlMode::kRestartRequired;
  }
  const bool fjt_active = *fjt_state == "active";
  const bool rolling_active = *rolling_state == "active";
  const bool fjt_inactive = *fjt_state == "inactive";
  const bool rolling_inactive = *rolling_state == "inactive";
  if (fjt_active && rolling_inactive) {
    return ControlMode::kFjtReady;
  }
  if (fjt_inactive && rolling_active) {
    return ControlMode::kRollingReady;
  }
  if (fjt_inactive && rolling_inactive) {
    return ControlMode::kDisabled;
  }
  return ControlMode::kRestartRequired;
}

EnableManagerController::ModeSwitchState
EnableManagerController::classifyModeSwitchState(
  const controller_manager_msgs::srv::ListControllers::Response & states,
  ControlMode source, ControlMode target) const
{
  const ControlMode observed = detectControlMode(states);
  if (observed == target) {
    return ModeSwitchState::kTargetActive;
  }
  if (observed == source) {
    return ModeSwitchState::kSourcePreserved;
  }
  if (observed == ControlMode::kDisabled) {
    return ModeSwitchState::kAllInactive;
  }
  return ModeSwitchState::kAmbiguous;
}

bool EnableManagerController::stableActualInterval(
  const ActualSample & previous, const ActualSample & current) const noexcept
{
  if (
    current.steady_time_ns <= previous.steady_time_ns ||
    current.steady_time_ns - previous.steady_time_ns >
    mode_switch_maximum_sample_period_ns_)
  {
    return false;
  }
  const double elapsed_seconds = static_cast<double>(
    current.steady_time_ns - previous.steady_time_ns) / 1.0e9;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (
      !std::isfinite(previous.positions[axis]) ||
      !std::isfinite(current.positions[axis]) ||
      std::abs(current.positions[axis] - previous.positions[axis]) /
      elapsed_seconds > mode_switch_stable_velocity_thresholds_[axis])
    {
      return false;
    }
  }
  return true;
}

bool EnableManagerController::sourceWithinTakeoverTolerance(
  const std::array<double, kAxisCount> & source,
  const std::array<double, kAxisCount> & actual) const noexcept
{
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (
      !std::isfinite(source[axis]) || !std::isfinite(actual[axis]) ||
      std::abs(source[axis] - actual[axis]) >
      mode_switch_takeover_tolerances_[axis])
    {
      return false;
    }
  }
  return true;
}

EnableManagerController::SwitchResult EnableManagerController::switchJtc(bool activate)
{
  bool expected = false;
  if (!switch_in_progress_.compare_exchange_strong(expected, true)) {
    return SwitchResult::kAmbiguous;
  }
  const auto clear_in_progress = [this]() {
      switch_in_progress_.store(false, std::memory_order_release);
    };
  if (
    !switch_client_->wait_for_service(std::chrono::milliseconds(0)) ||
    !list_client_->wait_for_service(std::chrono::milliseconds(0)))
  {
    clear_in_progress();
    return SwitchResult::kAmbiguous;
  }

  auto list_request =
    std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto list_future = list_client_->async_send_request(list_request);
  const auto list_wait_status = list_future.wait_for(
    std::chrono::duration<double>(controller_switch_timeout_seconds_));
  if (list_wait_status != std::future_status::ready) {
    clear_in_progress();
    return SwitchResult::kAmbiguous;
  }
  const auto list_response = list_future.get();
  if (list_response == nullptr) {
    clear_in_progress();
    return SwitchResult::kAmbiguous;
  }

  if (!activate) {
    bool has_active_registered_controller = false;
    for (const auto & controller : list_response->controller) {
      if (controller.state != "active") {
        continue;
      }
      if (std::find(
          motion_controller_names_.begin(), motion_controller_names_.end(),
          controller.name) != motion_controller_names_.end())
      {
        has_active_registered_controller = true;
        break;
      }
    }
    if (!has_active_registered_controller) {
      const bool all_inactive =
        registeredMotionControllerStateMatches(*list_response, false);
      clear_in_progress();
      if (all_inactive) {
        return SwitchResult::kSuccess;
      }
      return SwitchResult::kFailed;
    }
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  const SwitchResult plan_result =
    buildMotionControllerSwitchRequest(*list_response, activate, *request);
  if (plan_result != SwitchResult::kSuccess) {
    clear_in_progress();
    return plan_result;
  }
  if (request->activate_controllers.empty() && request->deactivate_controllers.empty()) {
    const bool state_matches =
      registeredMotionControllerStateMatches(*list_response, activate);
    clear_in_progress();
    return state_matches ? SwitchResult::kSuccess : SwitchResult::kFailed;
  }
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  const std::int64_t switch_timeout_ns =
    rclcpp::Duration::from_seconds(controller_switch_timeout_seconds_).nanoseconds();
  request->timeout.sec = static_cast<std::int32_t>(switch_timeout_ns / 1000000000LL);
  request->timeout.nanosec =
    static_cast<std::uint32_t>(switch_timeout_ns % 1000000000LL);
  auto future = switch_client_->async_send_request(request);
  const auto wait_status = future.wait_for(
    std::chrono::duration<double>(controller_switch_timeout_seconds_));
  SwitchResult result = SwitchResult::kAmbiguous;
  if (wait_status == std::future_status::ready) {
    const auto switch_response = future.get();
    if (switch_response != nullptr && switch_response->ok) {
      auto verification_request =
        std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
      auto verification_future = list_client_->async_send_request(verification_request);
      const auto verification_wait = verification_future.wait_for(
        std::chrono::duration<double>(controller_switch_timeout_seconds_));
      if (verification_wait == std::future_status::ready) {
        const auto verification_response = verification_future.get();
        result = verification_response != nullptr &&
          registeredMotionControllerStateMatches(*verification_response, activate) ?
          SwitchResult::kSuccess : SwitchResult::kAmbiguous;
      }
    } else {
      result = SwitchResult::kFailed;
    }
  }
  clear_in_progress();
  return result;
}

void EnableManagerController::handleNonRtFaultStop()
{
  if (switch_in_progress_.load(std::memory_order_acquire)) {
    return;
  }
  if (!emergency_jtc_deactivate_request_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  if (!jtc_deactivation_required_.load(std::memory_order_acquire)) {
    return;
  }
  if (switchJtc(false) != SwitchResult::kSuccess) {
    restart_required_.store(true, std::memory_order_release);
  } else {
    jtc_deactivation_required_.store(false, std::memory_order_release);
  }
}

void EnableManagerController::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_node()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/robot/rt_control/enable_manager";
  status.hardware_id = "robot-001";
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (phase == Phase::kFailed || restart_required_.load(std::memory_order_acquire)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "failed";
  } else if (
    phase != Phase::kIdle && phase != Phase::kEnabled && phase != Phase::kInactive)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "operation in progress";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "operational";
  }
  const std::int8_t failed_joint = last_failed_joint_.load(std::memory_order_acquire);
  const auto add_value = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(std::move(item));
    };
  add_value("state", phaseName(phase));
  add_value(
    "failed_batch", std::to_string(last_failed_batch_.load(std::memory_order_acquire)));
  add_value("failed_joint", failed_joint >= 0 ? kJointNames[failed_joint] : "");
  add_value(
    "failed_status_word",
    std::to_string(last_failed_status_word_.load(std::memory_order_acquire)));
  add_value("stage", stageName(last_failure_stage_.load(std::memory_order_acquire)));
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

void EnableManagerController::publishResult(
  ResultSlot & slot, bool ok, Stage stage, std::int8_t failed_batch,
  std::int8_t failed_joint, std::uint16_t status_word)
{
  slot.ok.store(ok, std::memory_order_relaxed);
  slot.failed_batch.store(failed_batch, std::memory_order_relaxed);
  slot.failed_joint.store(failed_joint, std::memory_order_relaxed);
  slot.status_word.store(status_word, std::memory_order_relaxed);
  slot.stage.store(stage, std::memory_order_relaxed);
  slot.ready.store(true, std::memory_order_release);
}

void EnableManagerController::recordFailure(
  Stage stage, std::int8_t failed_batch, std::int8_t failed_joint,
  std::uint16_t status_word)
{
  last_failure_stage_.store(stage, std::memory_order_relaxed);
  last_failed_batch_.store(failed_batch, std::memory_order_relaxed);
  last_failed_joint_.store(failed_joint, std::memory_order_relaxed);
  last_failed_status_word_.store(status_word, std::memory_order_release);
}

void EnableManagerController::clearFailure()
{
  last_failure_stage_.store(Stage::kSuccess, std::memory_order_relaxed);
  last_failed_batch_.store(-1, std::memory_order_relaxed);
  last_failed_joint_.store(-1, std::memory_order_relaxed);
  last_failed_status_word_.store(0U, std::memory_order_release);
}

void EnableManagerController::startDownward(Phase phase, std::int64_t now_ns)
{
  downward_stage_ = 0U;
  stage_deadline_ns_ =
    now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
  phase_.store(phase, std::memory_order_release);
}

void EnableManagerController::updateDownward(std::int64_t now_ns)
{
  const Phase active_phase = phase_.load(std::memory_order_acquire);
  bool any_operation_enabled = false;
  bool any_switched_on = false;
  bool all_nonfault_terminal = true;
  bool any_fault = false;
  std::int8_t first_nonterminal = -1;

  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    std::uint16_t command = 0x0000U;
    switch (state) {
      case DriveState::kOperationEnabled:
        command = 0x0007U;
        any_operation_enabled = true;
        all_nonfault_terminal = false;
        break;
      case DriveState::kSwitchedOn:
        command = 0x0006U;
        any_switched_on = true;
        all_nonfault_terminal = false;
        break;
      case DriveState::kReadyToSwitchOn:
        command = 0x0000U;
        all_nonfault_terminal =
          all_nonfault_terminal && isConfirmedDisableTerminal(axis, state);
        break;
      case DriveState::kQuickStopActive:
      case DriveState::kNotReady:
      case DriveState::kUnknown:
        command = 0x0000U;
        all_nonfault_terminal = false;
        break;
      case DriveState::kFaultReactionActive:
      case DriveState::kFault:
        any_fault = true;
        command = 0x0000U;
        break;
      case DriveState::kSwitchOnDisabled:
        command = 0x0000U;
        break;
    }
    if (!isConfirmedDisableTerminal(axis, state) && !isFaultState(state) &&
      first_nonterminal < 0)
    {
      first_nonterminal = static_cast<std::int8_t>(axis);
    }
    command_interfaces_[axis].set_value(command);
  }

  bool advance_stage = false;
  if (downward_stage_ == 0U) {
    advance_stage = !any_operation_enabled;
  } else if (downward_stage_ == 1U) {
    advance_stage = !any_operation_enabled && !any_switched_on;
  } else if (all_nonfault_terminal) {
    finishDownward();
    return;
  }

  if (advance_stage) {
    ++downward_stage_;
    stage_deadline_ns_ =
      now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
    if (downward_stage_ > 2U && all_nonfault_terminal) {
      finishDownward();
    }
    return;
  }

  if (now_ns >= stage_deadline_ns_) {
    if (first_nonterminal < 0 && any_fault) {
      for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
        if (isFaultState(decodeState(status_words_[axis]))) {
          first_nonterminal = static_cast<std::int8_t>(axis);
          break;
        }
      }
    }
    if (active_phase == Phase::kResetDisabling) {
      const std::uint16_t status =
        first_nonterminal >= 0 ? status_words_[first_nonterminal] : 0U;
      recordFailure(Stage::kFaultResetTimeout, -1, first_nonterminal, status);
      publishResult(reset_result_, false, Stage::kFaultResetTimeout, -1, first_nonterminal, status);
      setAllControlWords(0x0000U);
      phase_.store(Phase::kFailed, std::memory_order_release);
      owner_.store(Owner::kNone, std::memory_order_release);
      return;
    }
    if (primary_failure_stage_ == Stage::kSuccess) {
      primary_failure_stage_ = any_fault ? Stage::kFaultRequiresReset : Stage::kDisableTimeout;
      primary_failed_joint_ = first_nonterminal;
      primary_failed_status_word_ =
        first_nonterminal >= 0 ? status_words_[first_nonterminal] : 0U;
      recordFailure(
        primary_failure_stage_, -1, primary_failed_joint_, primary_failed_status_word_);
    }
    if (phase_.load(std::memory_order_acquire) != Phase::kEmergencyDisable) {
      interrupted_owner_ = owner_.load(std::memory_order_acquire);
      stage_deadline_ns_ =
        now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
      emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
      phase_.store(Phase::kEmergencyQuickStop, std::memory_order_release);
    } else {
      finishDownward();
    }
  }
}

void EnableManagerController::startEmergency(
  Stage stage, std::int8_t failed_joint, std::uint16_t status_word,
  std::int64_t now_ns)
{
  interrupted_owner_ = owner_.load(std::memory_order_acquire);
  if (interrupted_owner_ == Owner::kMode) {
    mode_abort_requested_.store(true, std::memory_order_release);
  }
  primary_failure_stage_ = stage;
  primary_failed_batch_ =
    interrupted_owner_ == Owner::kEnable ? static_cast<std::int8_t>(current_batch_) : -1;
  primary_failed_joint_ = failed_joint;
  primary_failed_status_word_ = status_word;
  recordFailure(stage, primary_failed_batch_, failed_joint, status_word);
  owner_.store(Owner::kInternal, std::memory_order_release);
  stage_deadline_ns_ =
    now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
  emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
  phase_.store(Phase::kEmergencyQuickStop, std::memory_order_release);
}

void EnableManagerController::updateEmergencyQuickStop(std::int64_t now_ns)
{
  bool any_operation_enabled = false;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    std::uint16_t command = 0x0000U;
    if (state == DriveState::kOperationEnabled || state == DriveState::kQuickStopActive) {
      command = 0x0002U;
    } else if (state == DriveState::kSwitchedOn) {
      command = 0x0006U;
    }
    any_operation_enabled = any_operation_enabled || state == DriveState::kOperationEnabled;
    command_interfaces_[axis].set_value(command);
  }
  if (!any_operation_enabled || now_ns >= stage_deadline_ns_) {
    startDownward(Phase::kEmergencyDisable, now_ns);
  }
}

void EnableManagerController::finishDownward()
{
  const Phase completed_phase = phase_.load(std::memory_order_acquire);
  bool any_fault = false;
  bool all_terminal = true;
  std::int8_t first_fault = -1;
  std::int8_t first_nonterminal = -1;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    if (isFaultState(state)) {
      any_fault = true;
      if (first_fault < 0) {
        first_fault = static_cast<std::int8_t>(axis);
      }
    } else if (!isConfirmedDisableTerminal(axis, state)) {
      all_terminal = false;
      if (first_nonterminal < 0) {
        first_nonterminal = static_cast<std::int8_t>(axis);
      }
    }
  }

  if (completed_phase == Phase::kStartupSanitizing) {
    current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
    if (primary_failure_stage_ != Stage::kSuccess) {
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else if (any_fault) {
      recordFailure(Stage::kFaultRequiresReset, -1, first_fault, status_words_[first_fault]);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else {
      phase_.store(Phase::kIdle, std::memory_order_release);
    }
    owner_.store(Owner::kNone, std::memory_order_release);
    return;
  }

  if (completed_phase == Phase::kDisabling) {
    current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
    if (any_fault) {
      publishResult(
        disable_result_, false, Stage::kFaultRequiresReset, -1, first_fault,
        status_words_[first_fault]);
      recordFailure(Stage::kFaultRequiresReset, -1, first_fault, status_words_[first_fault]);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else if (jtc_deactivate_failed_.load(std::memory_order_acquire)) {
      publishResult(disable_result_, false, Stage::kJtcDeactivateFailed);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else if (primary_failure_stage_ != Stage::kSuccess) {
      publishResult(
        disable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else {
      clearFailure();
      publishResult(disable_result_, true, Stage::kSuccess);
      phase_.store(Phase::kIdle, std::memory_order_release);
    }
    owner_.store(Owner::kNone, std::memory_order_release);
    return;
  }

  if (completed_phase == Phase::kRollback) {
    current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
    publishResult(
      enable_result_, false, primary_failure_stage_, primary_failed_batch_,
      primary_failed_joint_, primary_failed_status_word_);
    const bool restart = restart_required_.load(std::memory_order_acquire);
    phase_.store(any_fault || restart ? Phase::kFailed : Phase::kIdle, std::memory_order_release);
    owner_.store(Owner::kNone, std::memory_order_release);
    return;
  }

  if (completed_phase == Phase::kResetDisabling) {
    current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
    if (any_fault || !all_terminal) {
      const std::int8_t failed_joint = first_fault >= 0 ? first_fault : first_nonterminal;
      const std::uint16_t status_word =
        failed_joint >= 0 ? status_words_[failed_joint] : 0U;
      publishResult(reset_result_, false, Stage::kFaultResetTimeout, -1, failed_joint, status_word);
      recordFailure(Stage::kFaultResetTimeout, -1, failed_joint, status_word);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else {
      clearFailure();
      publishResult(reset_result_, true, Stage::kSuccess);
      phase_.store(Phase::kIdle, std::memory_order_release);
    }
    owner_.store(Owner::kNone, std::memory_order_release);
    return;
  }

  if (completed_phase == Phase::kEmergencyDisable) {
    current_control_mode_.store(
      restart_required_.load(std::memory_order_acquire) ?
      ControlMode::kRestartRequired : ControlMode::kDisabled,
      std::memory_order_release);
    if (interrupted_owner_ == Owner::kEnable) {
      publishResult(
        enable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
    } else if (interrupted_owner_ == Owner::kDisable) {
      publishResult(
        disable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
    }
    const bool restart = restart_required_.load(std::memory_order_acquire);
    phase_.store(
      any_fault || !all_terminal || restart ? Phase::kFailed : Phase::kIdle,
      std::memory_order_release);
    owner_.store(Owner::kNone, std::memory_order_release);
  }
}

void EnableManagerController::updateEnable(std::int64_t now_ns)
{
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (disable_request_.load(std::memory_order_acquire)) {
    enable_preempt_requested_ = true;
  }

  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    if (isFaultState(decodeState(status_words_[axis]))) {
      disable_request_.store(false, std::memory_order_release);
      startEmergency(
        Stage::kFaultDetected, static_cast<std::int8_t>(axis), status_words_[axis], now_ns);
      return;
    }
  }

  if (phase == Phase::kInterBatchDelay) {
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      command_interfaces_[axis].set_value(
        decodeState(status_words_[axis]) == DriveState::kOperationEnabled ? 0x000FU : 0x0000U);
    }
    if (enable_preempt_requested_) {
      disable_request_.store(false, std::memory_order_release);
      publishResult(enable_result_, false, Stage::kPreemptedByDisable);
      owner_.store(Owner::kDisable, std::memory_order_release);
      primary_failure_stage_ = Stage::kSuccess;
      startDownward(Phase::kDisabling, now_ns);
    } else if (now_ns >= stage_deadline_ns_) {
      ++current_batch_;
      stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(batch_timeout_seconds_ * 1e9);
      phase_.store(Phase::kEnabling, std::memory_order_release);
    }
    return;
  }

  bool batch_complete = true;
  std::int8_t invalid_axis = -1;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    bool is_current = false;
    bool is_previous = false;
    for (std::size_t batch = 0; batch <= current_batch_; ++batch) {
      for (std::size_t item = 0; item < kBatchSizes[batch]; ++item) {
        if (static_cast<std::size_t>(kEnableBatches[batch][item]) == axis) {
          is_current = batch == current_batch_;
          is_previous = batch < current_batch_;
        }
      }
    }

    const DriveState state = decodeState(status_words_[axis]);
    std::uint16_t command = 0x0000U;
    if (is_previous) {
      command = 0x000FU;
      if (state != DriveState::kOperationEnabled && invalid_axis < 0) {
        invalid_axis = static_cast<std::int8_t>(axis);
      }
    } else if (is_current) {
      switch (state) {
        case DriveState::kSwitchOnDisabled:
          command = 0x0006U;
          batch_complete = false;
          break;
        case DriveState::kReadyToSwitchOn:
          command = 0x0007U;
          batch_complete = false;
          break;
        case DriveState::kSwitchedOn:
          command = 0x000FU;
          batch_complete = false;
          break;
        case DriveState::kOperationEnabled:
          command = 0x000FU;
          break;
        default:
          command = 0x0000U;
          batch_complete = false;
          if (invalid_axis < 0) {
            invalid_axis = static_cast<std::int8_t>(axis);
          }
          break;
      }
    }
    command_interfaces_[axis].set_value(command);
  }

  if (invalid_axis >= 0) {
    primary_failure_stage_ = Stage::kEnableInvalidState;
    primary_failed_batch_ = static_cast<std::int8_t>(current_batch_);
    primary_failed_joint_ = invalid_axis;
    primary_failed_status_word_ = status_words_[invalid_axis];
    recordFailure(
      primary_failure_stage_, primary_failed_batch_, invalid_axis,
      primary_failed_status_word_);
    if (enable_preempt_requested_) {
      publishResult(
        enable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
      owner_.store(Owner::kDisable, std::memory_order_release);
      disable_request_.store(false, std::memory_order_release);
      startDownward(Phase::kDisabling, now_ns);
    } else {
      startDownward(Phase::kRollback, now_ns);
    }
    return;
  }

  if (batch_complete) {
    if (enable_preempt_requested_) {
      disable_request_.store(false, std::memory_order_release);
      publishResult(enable_result_, false, Stage::kPreemptedByDisable);
      owner_.store(Owner::kDisable, std::memory_order_release);
      primary_failure_stage_ = Stage::kSuccess;
      startDownward(Phase::kDisabling, now_ns);
    } else if (current_batch_ + 1U == kBatchCount) {
      enable_hardware_ready_.store(true, std::memory_order_release);
      phase_.store(Phase::kJtcActivating, std::memory_order_release);
    } else {
      stage_deadline_ns_ =
        now_ns + static_cast<std::int64_t>(inter_batch_delay_seconds_ * 1e9);
      phase_.store(Phase::kInterBatchDelay, std::memory_order_release);
    }
    return;
  }

  if (now_ns >= stage_deadline_ns_) {
    std::int8_t failed_axis = kEnableBatches[current_batch_][0];
    for (std::size_t item = 0; item < kBatchSizes[current_batch_]; ++item) {
      const auto axis = static_cast<std::size_t>(kEnableBatches[current_batch_][item]);
      if (decodeState(status_words_[axis]) != DriveState::kOperationEnabled) {
        failed_axis = static_cast<std::int8_t>(axis);
        break;
      }
    }
    primary_failure_stage_ = Stage::kEnableBatchTimeout;
    primary_failed_batch_ = static_cast<std::int8_t>(current_batch_);
    primary_failed_joint_ = failed_axis;
    primary_failed_status_word_ = status_words_[failed_axis];
    recordFailure(
      primary_failure_stage_, primary_failed_batch_, failed_axis,
      primary_failed_status_word_);
    if (enable_preempt_requested_) {
      publishResult(
        enable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
      owner_.store(Owner::kDisable, std::memory_order_release);
      disable_request_.store(false, std::memory_order_release);
      primary_failure_stage_ = Stage::kSuccess;
      startDownward(Phase::kDisabling, now_ns);
    } else {
      startDownward(Phase::kRollback, now_ns);
    }
  }
}

void EnableManagerController::updateReset(std::int64_t now_ns)
{
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
    setAllControlWords(0x0000U);
    publishResult(reset_result_, false, Stage::kPreemptedByDisable);
    owner_.store(Owner::kDisable, std::memory_order_release);
    primary_failure_stage_ = Stage::kSuccess;
    startDownward(Phase::kDisabling, now_ns);
    return;
  }

  if (phase == Phase::kResetLow) {
    setAllControlWords(0x0000U);
    stage_deadline_ns_ =
      now_ns + static_cast<std::int64_t>(fault_reset_timeout_seconds_ * 1e9);
    phase_.store(Phase::kResetHigh, std::memory_order_release);
    return;
  }

  bool all_reset_targets_left_fault = true;
  std::int8_t first_pending = -1;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    std::uint16_t command = 0x0000U;
    if (reset_targets_[axis] && state == DriveState::kFault) {
      command = 0x0080U;
    }
    command_interfaces_[axis].set_value(command);

    if (isFaultState(state)) {
      all_reset_targets_left_fault = false;
      if (first_pending < 0) {
        first_pending = static_cast<std::int8_t>(axis);
      }
      continue;
    }

    if (reset_targets_[axis]) {
      reset_targets_[axis] = false;
    }
  }

  if (all_reset_targets_left_fault) {
    startDownward(Phase::kResetDisabling, now_ns);
  } else if (now_ns >= stage_deadline_ns_) {
    const std::uint16_t status = first_pending >= 0 ? status_words_[first_pending] : 0U;
    recordFailure(Stage::kFaultResetTimeout, -1, first_pending, status);
    publishResult(reset_result_, false, Stage::kFaultResetTimeout, -1, first_pending, status);
    setAllControlWords(0x0000U);
    phase_.store(Phase::kFailed, std::memory_order_release);
    owner_.store(Owner::kNone, std::memory_order_release);
  }
}

void EnableManagerController::setAllControlWords(std::uint16_t control_word)
{
  for (auto & interface : command_interfaces_) {
    interface.set_value(control_word);
  }
}

bool EnableManagerController::allAxesInState(DriveState state) const
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    if (decodeState(status_words_[axis]) != state) {
      return false;
    }
  }
  return true;
}

std::int8_t EnableManagerController::firstAxisNotInState(DriveState state) const
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    if (decodeState(status_words_[axis]) != state) {
      return static_cast<std::int8_t>(axis);
    }
  }
  return -1;
}

void EnableManagerController::refreshStatusWords()
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const double value = state_interfaces_[axis].get_value();
    status_words_[axis] =
      std::isfinite(value) ? static_cast<std::uint16_t>(value) : 0xFFFFU;
  }
}

std::uint64_t EnableManagerController::steadyNowNs() noexcept
{
  const auto elapsed = std::chrono::steady_clock::now().time_since_epoch();
  const auto nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
  return nanoseconds < 0 ? 0U : static_cast<std::uint64_t>(nanoseconds);
}

std::uint64_t EnableManagerController::encodeDouble(double value) noexcept
{
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

double EnableManagerController::decodeDouble(std::uint64_t bits) noexcept
{
  double value = 0.0;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool EnableManagerController::isZeroIdentifier(
  const std::array<std::uint8_t, 16U> & identifier) noexcept
{
  return std::all_of(
    identifier.begin(), identifier.end(),
    [](std::uint8_t byte) {return byte == 0U;});
}

void EnableManagerController::populateModeError(
  robot_system_interfaces::msg::ErrorInfo & error, std::uint8_t result)
{
  using ErrorCode = robot_system_interfaces::msg::ErrorCode;
  using ErrorInfo = robot_system_interfaces::msg::ErrorInfo;
  using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
  std::uint32_t code = ErrorCode::INTERNAL_ERROR;
  switch (result) {
    case ServiceResult::NONE:
      code = ErrorCode::SUCCESS;
      break;
    case ServiceResult::WRONG_PROTOCOL:
      code = ErrorCode::VERSION_MISMATCH;
      break;
    case ServiceResult::WRONG_REQUEST:
    case ServiceResult::WRONG_CLIENT:
    case ServiceResult::AXIS_SET_MISMATCH:
      code = ErrorCode::INVALID_GOAL;
      break;
    case ServiceResult::WRONG_BOOT:
    case ServiceResult::WRONG_SESSION:
      code = ErrorCode::RETRYABLE_INVALID_GOAL;
      break;
    case ServiceResult::NOT_ENABLED:
    case ServiceResult::SESSION_BUSY:
    case ServiceResult::SESSION_EXISTS:
    case ServiceResult::SOURCE_STATE_STALE:
    case ServiceResult::SOURCE_MOVING:
    case ServiceResult::FEEDBACK_STALE:
    case ServiceResult::SWITCH_TIMEOUT:
    case ServiceResult::NOT_READY:
      code = ErrorCode::NOT_READY;
      break;
    case ServiceResult::WRONG_MODE:
    case ServiceResult::SWITCH_REJECTED:
      code = ErrorCode::GOAL_REJECTED;
      break;
    case ServiceResult::TAKEOVER_MISMATCH:
      code = ErrorCode::RT_TOLERANCE_VIOLATED;
      break;
    case ServiceResult::UNSAFE_HOLD:
      code = ErrorCode::SAFETY_DENIED;
      break;
    case ServiceResult::LIMITS_UNAVAILABLE:
    case ServiceResult::RESTART_REQUIRED:
      code = ErrorCode::VERSION_MISMATCH;
      break;
    default:
      break;
  }
  error.code = code;
  error.retryable = code != ErrorCode::SUCCESS && ((code / 100U) % 10U) == 1U;
  error.severity = code == ErrorCode::SUCCESS ?
    ErrorInfo::OK : (error.retryable ? ErrorInfo::WARN : ErrorInfo::FAULT);
  error.source = "rt_control";
  error.message = code == ErrorCode::SUCCESS ?
    "mode request accepted" : "mode request rejected";
  error.detail = "rolling_service_result=" + std::to_string(result);
}

EnableManagerController::DriveState EnableManagerController::decodeState(
  std::uint16_t status_word)
{
  if ((status_word & 0x004FU) == 0x0000U) {
    return DriveState::kNotReady;
  }
  if ((status_word & 0x004FU) == 0x0040U) {
    return DriveState::kSwitchOnDisabled;
  }
  if ((status_word & 0x006FU) == 0x0021U) {
    return DriveState::kReadyToSwitchOn;
  }
  if ((status_word & 0x006FU) == 0x0023U) {
    return DriveState::kSwitchedOn;
  }
  if ((status_word & 0x006FU) == 0x0027U) {
    return DriveState::kOperationEnabled;
  }
  if ((status_word & 0x006FU) == 0x0007U) {
    return DriveState::kQuickStopActive;
  }
  if ((status_word & 0x004FU) == 0x000FU) {
    return DriveState::kFaultReactionActive;
  }
  if ((status_word & 0x004FU) == 0x0008U) {
    return DriveState::kFault;
  }
  return DriveState::kUnknown;
}

bool EnableManagerController::isFaultState(DriveState state)
{
  return state == DriveState::kFault || state == DriveState::kFaultReactionActive;
}

bool EnableManagerController::isConfirmedDisableTerminal(std::size_t axis, DriveState state)
{
  if (state == DriveState::kSwitchOnDisabled) {
    return true;
  }
  const bool is_ti5_axis = axis == 1U || axis == 2U || axis == 7U || axis == 8U;
  return is_ti5_axis && state == DriveState::kReadyToSwitchOn;
}

const char * EnableManagerController::stageName(Stage stage)
{
  switch (stage) {
    case Stage::kSuccess: return "success";
    case Stage::kAlreadyEnabled: return "already_enabled";
    case Stage::kAlreadyDisabled: return "already_disabled";
    case Stage::kAlreadyClear: return "already_clear";
    case Stage::kOperationInProgress: return "operation_in_progress";
    case Stage::kControllerInactive: return "controller_inactive";
    case Stage::kPreemptedByDisable: return "preempted_by_disable";
    case Stage::kEnableBatchTimeout: return "enable_batch_timeout";
    case Stage::kEnableInvalidState: return "enable_invalid_state";
    case Stage::kFaultDetected: return "fault_detected";
    case Stage::kUnexpectedDriveState: return "unexpected_drive_state";
    case Stage::kDisableTimeout: return "disable_timeout";
    case Stage::kFaultRequiresReset: return "fault_requires_reset";
    case Stage::kFaultResetTimeout: return "fault_reset_timeout";
    case Stage::kJtcActivateFailed: return "jtc_activate_failed";
    case Stage::kJtcDeactivateFailed: return "jtc_deactivate_failed";
    case Stage::kControllerUpdateTimeout: return "controller_update_timeout";
    case Stage::kRestartRequired: return "restart_required";
  }
  return "unknown";
}

const char * EnableManagerController::phaseName(Phase phase)
{
  switch (phase) {
    case Phase::kInactive: return "INACTIVE";
    case Phase::kStartupSanitizing: return "STARTUP_SANITIZING";
    case Phase::kIdle: return "IDLE";
    case Phase::kEnabling: return "ENABLING";
    case Phase::kInterBatchDelay: return "INTER_BATCH_DELAY";
    case Phase::kJtcActivating: return "JTC_ACTIVATING";
    case Phase::kEnabled: return "ENABLED";
    case Phase::kJtcDeactivating: return "JTC_DEACTIVATING";
    case Phase::kDisabling: return "DISABLING";
    case Phase::kRollback: return "ROLLBACK";
    case Phase::kResetLow: return "RESET_LOW";
    case Phase::kResetHigh: return "RESET_HIGH";
    case Phase::kResetDisabling: return "RESET_DISABLING";
    case Phase::kEmergencyQuickStop: return "EMERGENCY_QUICK_STOP";
    case Phase::kEmergencyDisable: return "EMERGENCY_DISABLE";
    case Phase::kFailed: return "FAILED";
  }
  return "UNKNOWN";
}

}  // namespace enable_manager

PLUGINLIB_EXPORT_CLASS(
  enable_manager::EnableManagerController, controller_interface::ControllerInterface)
