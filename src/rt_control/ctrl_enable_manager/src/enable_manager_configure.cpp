// Parameters, interface claims and lifecycle configuration (non-realtime).
#include "ctrl_enable_manager/enable_manager_controller.hpp"

#include <algorithm>
#include <set>
#include <array>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/joint_control_mode.hpp"
#include "robot_system_interfaces/msg/error_code.hpp"

namespace ctrl_enable_manager
{

namespace
{

constexpr std::array<std::string_view, 4> kTopologyParameterNames{
  "managed_joints", "enable_batch_joint_names", "enable_batch_sizes",
  "ready_to_switch_on_disable_terminal_joints"};

constexpr std::uint8_t topologyParameterBit(std::size_t index)
{
  return static_cast<std::uint8_t>(1U << index);
}

constexpr std::uint8_t kAllTopologyParametersMask =
  static_cast<std::uint8_t>((1U << kTopologyParameterNames.size()) - 1U);

}  // namespace

controller_interface::CallbackReturn EnableManagerController::on_init()
{
  auto_declare<std::vector<std::string>>("managed_joints", {});
  auto_declare<std::vector<std::int64_t>>("expected_modes", {});
  auto_declare<std::vector<std::string>>("ready_state_interfaces", {});
  auto_declare<std::vector<std::string>>("feedback_age_interfaces", {});
  auto_declare<std::vector<std::string>>("al_state_interfaces", {});
  // User 2026-11-03: 500 ms for every scope (normal ages ~1 ms EtherCAT, 6-20 ms DaMiao).
  auto_declare<double>("feedback_age_limit_ms", 500.0);
  auto_declare<double>("ready_hold_time", 0.05);
  auto_declare<std::string>("motion_fault_topic", "");
  auto_declare<std::vector<std::string>>("enable_batch_joint_names", {});
  auto_declare<std::vector<std::int64_t>>("enable_batch_sizes", {});
  auto_declare<std::vector<std::string>>("ready_to_switch_on_disable_terminal_joints", {});
  auto_declare<std::vector<std::string>>("handshake_devices", {});
  auto_declare<std::vector<std::string>>("diagnostic_joints", {});
  auto_declare<std::vector<std::string>>("diagnostic_joint_interfaces", {});

  auto_declare<double>("batch_timeout", 4.0);
  auto_declare<double>("disable_stage_timeout", 4.0);
  auto_declare<double>("inter_batch_delay", 0.2);
  auto_declare<double>("fault_reset_timeout", 4.0);
  auto_declare<double>("controller_switch_timeout", 4.0);
  auto_declare<int>("service_result_timeout_ms", 30000);
  auto_declare<std::string>("jtc_name", "whole_body_jtc");
  // Further controllers switched in the same request as jtc_name (e.g. a head next to the arms).
  auto_declare<std::vector<std::string>>("companion_controllers", {});
  auto_declare<bool>("enable_only", false);
  auto_declare<std::string>("disable_terminal_policy", "joint_list");
  // Runtime-LOG: copy the state machine into a shared-memory ring every cycle (optional).
  auto_declare<bool>("runtime_log_trace", false);
  auto_declare<bool>("motion_mode_switching", false);
  auto_declare<std::vector<std::string>>("motion_joints", {});
  auto_declare<std::vector<std::string>>(
    "motion_controller_names", {"whole_body_jtc", "rolling_trajectory_controller"});
  auto_declare<std::string>("default_motion_controller", "whole_body_jtc");
  auto_declare<std::string>("rolling_motion_controller", "rolling_trajectory_controller");
  auto_declare<int>("mode_switch_maximum_sample_period_ms", 2);
  auto_declare<int>("mode_switch_source_state_max_age_ms", 100);
  auto_declare<int>("mode_switch_stable_interval_count", 5);
  auto_declare<int>("mode_switch_timeout_ms", 500);
  auto_declare<double>("mode_switch_feedback_age_limit_ms", 500.0);
  auto_declare<std::vector<double>>(
    "mode_switch_stable_velocity_thresholds", std::vector<double>(kMotionAxisCount, 0.00872664626));
  auto_declare<std::vector<double>>(
    "mode_switch_takeover_tolerances", std::vector<double>(kMotionAxisCount, 0.00872664626));

  topology_parameters_frozen_.store(false, std::memory_order_release);
  topology_parameters_explicit_mask_.store(0U, std::memory_order_release);
  const auto & parameter_overrides =
    get_node()->get_node_parameters_interface()->get_parameter_overrides();
  for (std::size_t index = 0; index < kTopologyParameterNames.size(); ++index) {
    if (parameter_overrides.count(std::string{kTopologyParameterNames[index]}) != 0U) {
      topology_parameters_explicit_mask_.fetch_or(
        topologyParameterBit(index), std::memory_order_acq_rel);
    }
  }
  topology_parameter_callback_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return validateTopologyParameterUpdate(parameters);
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
EnableManagerController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(cia402_axes_.size());
  for (const auto & axis : cia402_axes_) {
    const auto & names = axis.get_command_interface_names();
    configuration.names.insert(configuration.names.end(), names.begin(), names.end());
  }
  for (const auto & device : handshake_devices_) {
    const auto names = device->command_interface_names();
    configuration.names.insert(configuration.names.end(), names.begin(), names.end());
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
EnableManagerController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(cia402_axes_.size());
  for (const auto & axis : cia402_axes_) {
    const auto & names = axis.get_state_interface_names();
    configuration.names.insert(configuration.names.end(), names.begin(), names.end());
  }
  if (motion_mode_switching_) {
    configuration.names.reserve(configuration.names.size() + kMotionAxisCount + 1U);
    for (const auto & joint : motion_joint_names_) {
      configuration.names.emplace_back(joint + "/position");
    }
    configuration.names.emplace_back("ethercat_domain/process_data_age_ms");
  }
  if (!expected_modes_.empty()) {
    for (const auto & joint : joint_names_) {
      configuration.names.emplace_back(joint + "/mode_of_operation_display");
    }
  }
  configuration.names.insert(
    configuration.names.end(), ready_state_names_.begin(), ready_state_names_.end());
  for (const auto & device : handshake_devices_) {
    const auto names = device->state_interface_names();
    configuration.names.insert(configuration.names.end(), names.begin(), names.end());
  }
  configuration.names.insert(
    configuration.names.end(), diagnostic_state_names_.begin(), diagnostic_state_names_.end());
  for (const auto & name : offline_source_names_) {
    if (
      std::find(configuration.names.begin(), configuration.names.end(), name) ==
      configuration.names.end()) {
      configuration.names.push_back(name);
    }
  }
  return configuration;
}

controller_interface::CallbackReturn EnableManagerController::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!configureTopology()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  batch_timeout_seconds_ = get_node()->get_parameter("batch_timeout").as_double();
  disable_stage_timeout_seconds_ = get_node()->get_parameter("disable_stage_timeout").as_double();
  inter_batch_delay_seconds_ = get_node()->get_parameter("inter_batch_delay").as_double();
  fault_reset_timeout_seconds_ = get_node()->get_parameter("fault_reset_timeout").as_double();
  controller_switch_timeout_seconds_ =
    get_node()->get_parameter("controller_switch_timeout").as_double();
  const auto service_timeout = get_node()->get_parameter("service_result_timeout_ms").as_int();
  jtc_name_ = get_node()->get_parameter("jtc_name").as_string();
  enable_only_ = get_node()->get_parameter("enable_only").as_bool();
  motion_mode_switching_ = get_node()->get_parameter("motion_mode_switching").as_bool();
  const auto companions = get_node()->get_parameter("companion_controllers").as_string_array();
  switched_controllers_ = {jtc_name_};
  switched_controllers_.insert(switched_controllers_.end(), companions.begin(), companions.end());
  const bool companions_valid =
    companions.empty() ||
    (!enable_only_ && !motion_mode_switching_ &&
     std::none_of(
       companions.begin(), companions.end(), [](const auto & name) { return name.empty(); }) &&
     std::set<std::string>(switched_controllers_.begin(), switched_controllers_.end()).size() ==
       switched_controllers_.size());
  const auto modes = get_node()->get_parameter("expected_modes").as_integer_array();
  if (
    !modes.empty() && (modes.size() != joint_names_.size() || motion_mode_switching_ ||
                       !std::all_of(modes.begin(), modes.end(), [](std::int64_t value) {
                         return value >= -128 && value <= 127 && value != 0;
                       }))) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "expected_modes must be empty or one nonzero int8 fixed mode per managed joint; incompatible "
      "with dynamic mode switching");
    return controller_interface::CallbackReturn::ERROR;
  }
  expected_modes_.assign(modes.begin(), modes.end());
  ready_state_names_ = get_node()->get_parameter("ready_state_interfaces").as_string_array();
  ready_hold_seconds_ = get_node()->get_parameter("ready_hold_time").as_double();
  ready_state_indices_.assign(ready_state_names_.size(), 0U);
  offline_source_names_ = get_node()->get_parameter("feedback_age_interfaces").as_string_array();
  age_source_count_ = offline_source_names_.size();
  const auto al_states = get_node()->get_parameter("al_state_interfaces").as_string_array();
  offline_source_names_.insert(offline_source_names_.end(), al_states.begin(), al_states.end());
  feedback_age_limit_ms_ = get_node()->get_parameter("feedback_age_limit_ms").as_double();
  offline_source_indices_.assign(offline_source_names_.size(), 0U);
  diagnostic_state_names_.clear();
  const auto diagnostic_joints = get_node()->get_parameter("diagnostic_joints").as_string_array();
  const auto diagnostic_interfaces =
    get_node()->get_parameter("diagnostic_joint_interfaces").as_string_array();
  for (const auto & joint : diagnostic_joints) {
    for (const auto & interface : diagnostic_interfaces) {
      diagnostic_state_names_.push_back(joint + "/" + interface);
    }
  }
  diagnostic_state_indices_.assign(diagnostic_state_names_.size(), 0U);
  diagnostic_state_values_ =
    std::make_unique<std::atomic<double>[]>(diagnostic_state_names_.size());
  actual_modes_.assign(modes.size(), 256);
  mode_state_indices_.assign(modes.size(), 0U);
  mode_confirmed_.assign(modes.size(), 0U);
  motion_joint_names_ = get_node()->get_parameter("motion_joints").as_string_array();
  motion_controller_names_ = get_node()->get_parameter("motion_controller_names").as_string_array();
  default_motion_controller_ = get_node()->get_parameter("default_motion_controller").as_string();
  rolling_motion_controller_ = get_node()->get_parameter("rolling_motion_controller").as_string();
  const auto maximum_sample_period_ms =
    get_node()->get_parameter("mode_switch_maximum_sample_period_ms").as_int();
  const auto source_state_max_age_ms =
    get_node()->get_parameter("mode_switch_source_state_max_age_ms").as_int();
  const auto stable_interval_count =
    get_node()->get_parameter("mode_switch_stable_interval_count").as_int();
  const auto mode_switch_timeout_ms = get_node()->get_parameter("mode_switch_timeout_ms").as_int();
  const double mode_switch_feedback_age_limit_ms =
    get_node()->get_parameter("mode_switch_feedback_age_limit_ms").as_double();
  const auto stable_velocity_thresholds =
    get_node()->get_parameter("mode_switch_stable_velocity_thresholds").as_double_array();
  const auto takeover_tolerances =
    get_node()->get_parameter("mode_switch_takeover_tolerances").as_double_array();

  std::vector<std::string> sorted_controller_names = motion_controller_names_;
  std::sort(sorted_controller_names.begin(), sorted_controller_names.end());
  const bool has_empty_controller_name = std::any_of(
    motion_controller_names_.begin(), motion_controller_names_.end(),
    [](const std::string & name) { return name.empty(); });
  const bool has_duplicate_controller_name =
    std::adjacent_find(sorted_controller_names.begin(), sorted_controller_names.end()) !=
    sorted_controller_names.end();
  const bool default_is_registered =
    std::find(
      motion_controller_names_.begin(), motion_controller_names_.end(),
      default_motion_controller_) != motion_controller_names_.end();
  const bool rolling_is_registered =
    std::find(
      motion_controller_names_.begin(), motion_controller_names_.end(),
      rolling_motion_controller_) != motion_controller_names_.end();
  std::vector<std::string> sorted_motion_joints = motion_joint_names_;
  std::sort(sorted_motion_joints.begin(), sorted_motion_joints.end());
  const bool motion_joints_valid =
    motion_joint_names_.size() == kMotionAxisCount &&
    std::adjacent_find(sorted_motion_joints.begin(), sorted_motion_joints.end()) ==
      sorted_motion_joints.end() &&
    std::all_of(
      motion_joint_names_.begin(), motion_joint_names_.end(), [this](const std::string & name) {
        return std::find(joint_names_.begin(), joint_names_.end(), name) != joint_names_.end();
      });
  const auto valid_threshold_array = [](const std::vector<double> & values) {
    return values.size() == kMotionAxisCount &&
           std::all_of(values.begin(), values.end(), [](double value) {
             return std::isfinite(value) && value >= 0.0;
           });
  };
  const bool motion_parameters_valid =
    !motion_mode_switching_ ||
    (!enable_only_ && motion_joints_valid && motion_controller_names_.size() == 2U &&
     !has_empty_controller_name && !has_duplicate_controller_name &&
     !default_motion_controller_.empty() && default_is_registered &&
     !rolling_motion_controller_.empty() && rolling_is_registered &&
     rolling_motion_controller_ != default_motion_controller_ &&
     jtc_name_ == default_motion_controller_ && maximum_sample_period_ms > 0 &&
     source_state_max_age_ms > 0 && stable_interval_count > 0 && mode_switch_timeout_ms > 0 &&
     std::isfinite(mode_switch_feedback_age_limit_ms) && mode_switch_feedback_age_limit_ms > 0.0 &&
     valid_threshold_array(stable_velocity_thresholds) &&
     valid_threshold_array(takeover_tolerances));

  // Timing values are checked once by the composer (enable policy schema; YAML load rejects
  // non-finite numbers).
  if (
    (enable_only_ ? !jtc_name_.empty() : jtc_name_.empty()) || !motion_parameters_valid ||
    !companions_valid) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid enable-manager controller parameter");
    return controller_interface::CallbackReturn::ERROR;
  }
  service_result_timeout_ = std::chrono::milliseconds(service_timeout);
  if (motion_mode_switching_) {
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
  }

  enable_callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  disable_callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  reset_callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  if (motion_mode_switching_) {
    mode_callback_group_ =
      get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }
  worker_callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // R-IN-18/19/20: public services; each result carries its DREE error.
  using ErrorCode = robot_system_interfaces::msg::ErrorCode;
  using RtEnable = robot_rt_control_interfaces::srv::RtEnable;
  const auto serve = [this](
                       const char * name, auto handler, std::uint32_t failure_code,
                       rclcpp::CallbackGroup::SharedPtr group) {
    return get_node()->create_service<RtEnable>(
      name,
      [this, name, handler, failure_code](
        const std::shared_ptr<RtEnable::Request> request,
        std::shared_ptr<RtEnable::Response> response) {
        (this->*handler)(request, response);
        reportResult(name, *response, failure_code);
      },
      rmw_qos_profile_services_default, group);
  };
  enable_service_ = serve(
    "/rt/enable", &EnableManagerController::handleEnable, ErrorCode::RT_ENABLE_FAILED,
    enable_callback_group_);
  disable_service_ = serve(
    "/rt/disable", &EnableManagerController::handleDisable, ErrorCode::RT_DISABLE_FAILED,
    disable_callback_group_);
  reset_service_ = serve(
    "/rt/reset_fault", &EnableManagerController::handleResetFault, ErrorCode::RT_RESET_FAILED,
    reset_callback_group_);
  if (motion_mode_switching_) {
    mode_service_ = get_node()->create_service<ModeService>(
      "/rt/arms/set_control_mode",
      std::bind(
        &EnableManagerController::handleSetMode, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, mode_callback_group_);
    rclcpp::SubscriptionOptions source_subscription_options;
    source_subscription_options.callback_group = worker_callback_group_;
    jtc_state_subscription_ =
      get_node()->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "/" + default_motion_controller_ + "/controller_state", robot_interfaces_qos::state(),
        std::bind(&EnableManagerController::handleJtcState, this, std::placeholders::_1),
        source_subscription_options);
    rolling_state_subscription_ =
      get_node()->create_subscription<robot_rt_control_interfaces::msg::RollingJointControlState>(
        "/rt/arms/rolling/state", robot_interfaces_qos::rolling_state(),
        std::bind(&EnableManagerController::handleRollingState, this, std::placeholders::_1),
        source_subscription_options);
  }
  list_client_ = get_node()->create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers", rmw_qos_profile_services_default,
    worker_callback_group_);
  switch_client_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller", rmw_qos_profile_services_default,
    worker_callback_group_);

  diagnostics_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", robot_interfaces_qos::diagnostic());
  const auto motion_fault_topic = get_node()->get_parameter("motion_fault_topic").as_string();
  motion_fault_subscription_.reset();
  if (!motion_fault_topic.empty()) {
    rclcpp::SubscriptionOptions fault_options;
    fault_options.callback_group = worker_callback_group_;
    motion_fault_subscription_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      motion_fault_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        // Only an enabled chassis can be stopped; stale reports never pre-arm a later enable.
        if (message->data && phase_.load(std::memory_order_acquire) == Phase::kEnabled) {
          motion_fault_request_.store(true, std::memory_order_release);
        }
      },
      fault_options);
  }
  worker_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&EnableManagerController::handleNonRtFaultStop, this),
    worker_callback_group_);
  diagnostics_timer_ = get_node()->create_wall_timer(
    std::chrono::seconds(1), std::bind(&EnableManagerController::publishDiagnostics, this),
    worker_callback_group_);

  active_.store(false, std::memory_order_release);
  phase_.store(Phase::kInactive, std::memory_order_release);
  owner_.store(Owner::kInternal, std::memory_order_release);
  current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(command_snapshot_mutex_);
    jtc_command_snapshot_ = CommandSnapshot{};
    rolling_command_snapshot_ = CommandSnapshot{};
  }
  topology_parameters_frozen_.store(true, std::memory_order_release);
  configureTrace();
  return controller_interface::CallbackReturn::SUCCESS;
}

void EnableManagerController::configureTrace()
{
  // Non-real-time: create, pre-fault and lock the segment. Failure only disables the trace.
  trace_.close();
  if (!get_node()->get_parameter("runtime_log_trace").as_bool()) {
    return;
  }
  const std::vector<std::string> fields(kTraceFields.begin(), kTraceFields.end());
  const std::string segment =
    "/runtime-log." + std::to_string(::getpid()) + "." + get_node()->get_name();
  std::string error;
  if (!trace_.create(
        segment, get_node()->get_name(), "enable_manager state machine", fields, 2000U, &error)) {
    RCLCPP_WARN(get_node()->get_logger(), "RUNTIME_LOG_TRACE_DISABLED %s", error.c_str());
    return;
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "RUNTIME_LOG_TRACE_CONFIGURED segment=%s fields=%zu", segment.c_str(),
    fields.size());
}

rcl_interfaces::msg::SetParametersResult EnableManagerController::validateTopologyParameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    if (
      parameter.get_name() == "enable_only" || parameter.get_name() == "jtc_name" ||
      parameter.get_name() == "companion_controllers" ||
      parameter.get_name() == "disable_terminal_policy" ||
      parameter.get_name() == "expected_modes" || parameter.get_name() == "motion_mode_switching" ||
      parameter.get_name() == "ready_state_interfaces" ||
      parameter.get_name() == "motion_fault_topic" || parameter.get_name() == "ready_hold_time" ||
      parameter.get_name() == "motion_joints" ||
      parameter.get_name() == "motion_controller_names" ||
      parameter.get_name() == "default_motion_controller" ||
      parameter.get_name() == "rolling_motion_controller" ||
      parameter.get_name() == "handshake_devices" || parameter.get_name() == "diagnostic_joints" ||
      parameter.get_name() == "diagnostic_joint_interfaces") {
      if (topology_parameters_frozen_.load(std::memory_order_acquire)) {
        result.successful = false;
        result.reason = "enable-manager controller policy is already configured";
        return result;
      }
    }
    const auto match = std::find(
      kTopologyParameterNames.begin(), kTopologyParameterNames.end(), parameter.get_name());
    if (match == kTopologyParameterNames.end()) {
      continue;
    }
    if (topology_parameters_frozen_.load(std::memory_order_acquire)) {
      result.successful = false;
      result.reason =
        "enable-manager topology is already configured; topology parameters are immutable";
      return result;
    }
    const auto index =
      static_cast<std::size_t>(std::distance(kTopologyParameterNames.begin(), match));
    topology_parameters_explicit_mask_.fetch_or(
      topologyParameterBit(index), std::memory_order_acq_rel);
  }
  return result;
}

bool EnableManagerController::configureTopology()
{
  auto explicit_mask = topology_parameters_explicit_mask_.load(std::memory_order_acquire);
  const auto device_names = get_node()->get_parameter("handshake_devices").as_string_array();
  handshake_devices_.clear();
  for (const auto & name : device_names) {
    handshake_devices_.push_back(std::make_unique<HandshakeDevice>(name));
  }
  // ROS parameter YAML cannot carry typed empty arrays, so a device-only manager is
  // expressed by giving none of the CiA402 topology parameters.
  if (!handshake_devices_.empty() && explicit_mask == 0U) {
    joint_names_.clear();
    enable_batches_.clear();
    ready_to_switch_on_disable_terminal_.clear();
    status_words_.clear();
    reset_targets_.clear();
    cia402_axes_.clear();
    return true;
  }
  const auto terminal_policy = get_node()->get_parameter("disable_terminal_policy").as_string();
  if (terminal_policy == "switch_on_disabled") {
    if (!get_node()
           ->get_parameter("ready_to_switch_on_disable_terminal_joints")
           .as_string_array()
           .empty()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Explicit disabled-terminal policy conflicts with joint exceptions");
      return false;
    }
    // This explicit policy has no per-joint exceptions. Avoid an untyped empty
    // array in ROS parameter YAML while preserving legacy explicit-list admission.
    explicit_mask |= topologyParameterBit(3U);
  } else if (terminal_policy != "joint_list") {
    RCLCPP_ERROR(get_node()->get_logger(), "Unknown disable_terminal_policy");
    return false;
  }
  if (explicit_mask != kAllTopologyParametersMask) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "managed_joints, enable_batch_joint_names, enable_batch_sizes, and "
      "ready_to_switch_on_disable_terminal_joints must all be explicitly configured");
    return false;
  }

  const auto managed_joints = get_node()->get_parameter("managed_joints").as_string_array();
  const auto flat_batch_joints =
    get_node()->get_parameter("enable_batch_joint_names").as_string_array();
  const auto batch_sizes = get_node()->get_parameter("enable_batch_sizes").as_integer_array();
  const auto ready_terminal_joints =
    get_node()->get_parameter("ready_to_switch_on_disable_terminal_joints").as_string_array();

  constexpr std::size_t kMaximumIndexedCount =
    static_cast<std::size_t>(std::numeric_limits<std::int8_t>::max());
  if (
    managed_joints.empty() || managed_joints.size() > kMaximumIndexedCount || batch_sizes.empty() ||
    batch_sizes.size() > kMaximumIndexedCount) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "managed_joints and enable_batch_sizes must each contain 1..127 entries");
    return false;
  }

  // The composer checks once that the managed joints are unique and that the batches (1..3
  // joints, enable policy schema) partition them exactly.
  std::unordered_map<std::string, std::size_t> axis_by_name;
  axis_by_name.reserve(managed_joints.size());
  for (std::size_t axis = 0; axis < managed_joints.size(); ++axis) {
    axis_by_name.emplace(managed_joints[axis], axis);
  }

  std::vector<std::vector<std::size_t>> candidate_batches;
  candidate_batches.reserve(batch_sizes.size());
  std::size_t flat_index = 0U;
  for (const auto batch_size : batch_sizes) {
    std::vector<std::size_t> batch;
    batch.reserve(static_cast<std::size_t>(batch_size));
    for (std::int64_t item = 0; item < batch_size; ++item) {
      batch.push_back(axis_by_name.at(flat_batch_joints[flat_index++]));
    }
    candidate_batches.push_back(std::move(batch));
  }

  std::vector<std::uint8_t> candidate_ready_terminal(managed_joints.size(), 0U);
  for (const auto & joint : ready_terminal_joints) {
    const auto axis = axis_by_name.find(joint);
    if (axis == axis_by_name.end() || candidate_ready_terminal[axis->second] != 0U) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "ready_to_switch_on_disable_terminal_joints must be a unique managed-joint subset");
      return false;
    }
    candidate_ready_terminal[axis->second] = 1U;
  }

  std::vector<hw_semantic_components::Cia402Axis> candidate_axes;
  candidate_axes.reserve(managed_joints.size());
  for (const auto & joint : managed_joints) {
    candidate_axes.emplace_back(joint);
  }

  joint_names_ = managed_joints;
  enable_batches_ = std::move(candidate_batches);
  ready_to_switch_on_disable_terminal_ = std::move(candidate_ready_terminal);
  status_words_.assign(managed_joints.size(), 0U);
  reset_targets_.assign(managed_joints.size(), 0U);
  cia402_axes_ = std::move(candidate_axes);
  return true;
}

}  // namespace ctrl_enable_manager
