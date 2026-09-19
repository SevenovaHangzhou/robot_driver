#include "damiao_head_controller/head_manager_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <future>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robot_interfaces_qos/profiles.hpp"

namespace damiao_head_controller
{
namespace
{

using CallbackReturn = controller_interface::CallbackReturn;

[[nodiscard]] bool exact_flag(double value) noexcept
{
  return value == 0.0 || value == 1.0;
}

[[nodiscard]] std::uint64_t exact_generation(double value)
{
  constexpr double kMaximumExactInteger{9007199254740991.0};
  if (!std::isfinite(value) || value < 0.0 || value > kMaximumExactInteger ||
    std::trunc(value) != value)
  {
    throw std::runtime_error{"DaMiao reset generation is not an exact nonnegative integer"};
  }
  return static_cast<std::uint64_t>(value);
}

diagnostic_msgs::msg::KeyValue key_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = std::move(key);
  item.value = std::move(value);
  return item;
}

}  // namespace

CallbackReturn HeadManagerController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("control_name", "damiao_head_control");
  auto_declare<std::string>("position_controller_name", "head_position_controller");
  auto_declare<std::string>("service_prefix", "/rt/head");
  auto_declare<int>("service_timeout_ms", 5000);
  auto_declare<int>("controller_switch_timeout_ms", 1000);
  auto_declare<double>("diagnostic_feedback_timeout_ms", 100.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn HeadManagerController::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    control_name_ = get_node()->get_parameter("control_name").as_string();
    position_controller_name_ =
      get_node()->get_parameter("position_controller_name").as_string();
    service_prefix_ = get_node()->get_parameter("service_prefix").as_string();
    const auto service_timeout_ms = get_node()->get_parameter("service_timeout_ms").as_int();
    const auto switch_timeout_ms =
      get_node()->get_parameter("controller_switch_timeout_ms").as_int();
    diagnostic_feedback_timeout_ms_ =
      get_node()->get_parameter("diagnostic_feedback_timeout_ms").as_double();
    if (joint_names_.size() != 2U || joint_names_[0].empty() || joint_names_[1].empty() ||
      joint_names_[0] == joint_names_[1] || control_name_.empty() ||
      position_controller_name_.empty() || service_prefix_.empty() ||
      service_prefix_.front() != '/' || service_timeout_ms <= 0 || switch_timeout_ms <= 0 ||
      !std::isfinite(diagnostic_feedback_timeout_ms_) || diagnostic_feedback_timeout_ms_ <= 0.0)
    {
      throw std::invalid_argument{"invalid DaMiao head-manager topology or timeout"};
    }
    service_timeout_ = std::chrono::milliseconds{service_timeout_ms};
    controller_switch_timeout_ = std::chrono::milliseconds{switch_timeout_ms};
    command_names_ = {
      control_name_ + "/enable_request", control_name_ + "/reset_generation"};
    state_names_ = {
      control_name_ + "/phase", control_name_ + "/fault_latched",
      control_name_ + "/handled_reset_generation", control_name_ + "/motion_allowed"};
    for (const auto & joint : joint_names_) {
      for (const char * interface : {
          "position", "velocity", "effort", "fault_code", "mos_temperature",
          "motor_temperature", "feedback_age_ms", "enabled"})
      {
        state_names_.push_back(joint + "/" + interface);
      }
    }

    service_callback_group_ =
      get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    worker_callback_group_ =
      get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    enable_service_ = get_node()->create_service<EnableService>(
      service_prefix_ + "/enable",
      std::bind(&HeadManagerController::handle_enable, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);
    disable_service_ = get_node()->create_service<EnableService>(
      service_prefix_ + "/disable",
      std::bind(&HeadManagerController::handle_disable, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);
    reset_service_ = get_node()->create_service<EnableService>(
      service_prefix_ + "/reset_fault",
      std::bind(&HeadManagerController::handle_reset, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);
    list_client_ = get_node()->create_client<ListControllers>(
      "/controller_manager/list_controllers", rmw_qos_profile_services_default,
      worker_callback_group_);
    switch_client_ = get_node()->create_client<SwitchController>(
      "/controller_manager/switch_controller", rmw_qos_profile_services_default,
      worker_callback_group_);
    diagnostics_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", robot_interfaces_qos::diagnostic());
    worker_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds{20}, std::bind(&HeadManagerController::handle_fault_stop, this),
      worker_callback_group_);
    diagnostics_timer_ = get_node()->create_wall_timer(
      std::chrono::seconds{1}, std::bind(&HeadManagerController::publish_diagnostics, this),
      worker_callback_group_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "DaMiao head manager configuration rejected: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
HeadManagerController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, command_names_};
}

controller_interface::InterfaceConfiguration HeadManagerController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, state_names_};
}

bool HeadManagerController::bind_interfaces()
{
  if (command_interfaces_.size() != commands_.size() || state_interfaces_.size() != states_.size()) {
    return false;
  }
  commands_.fill(nullptr);
  states_.fill(nullptr);
  for (std::size_t index{0U}; index < command_names_.size(); ++index) {
    for (auto & interface : command_interfaces_) {
      if (interface.get_name() == command_names_[index]) {
        if (commands_[index] != nullptr) {
          return false;
        }
        commands_[index] = &interface;
      }
    }
    if (commands_[index] == nullptr) {
      return false;
    }
  }
  for (std::size_t index{0U}; index < state_names_.size(); ++index) {
    for (auto & interface : state_interfaces_) {
      if (interface.get_name() == state_names_[index]) {
        if (states_[index] != nullptr) {
          return false;
        }
        states_[index] = &interface;
      }
    }
    if (states_[index] == nullptr) {
      return false;
    }
  }
  return true;
}

CallbackReturn HeadManagerController::on_activate(const rclcpp_lifecycle::State &)
{
  if (active_.load(std::memory_order_acquire) || !bind_interfaces()) {
    return CallbackReturn::ERROR;
  }
  try {
    const double phase_value = states_[0]->get_value();
    if (!std::isfinite(phase_value) || phase_value < 0.0 || phase_value > 6.0 ||
      std::trunc(phase_value) != phase_value)
    {
      throw std::runtime_error{"DaMiao head phase state is invalid"};
    }
    requested_enable_.store(false, std::memory_order_release);
    const auto generation = exact_generation(states_[2]->get_value());
    requested_reset_generation_.store(generation, std::memory_order_release);
    commands_[0]->set_value(0.0);
    commands_[1]->set_value(static_cast<double>(generation));
    operation_.store(Operation::none, std::memory_order_release);
    fault_stop_requested_.store(false, std::memory_order_release);
    position_controller_active_.store(false, std::memory_order_release);
    active_.store(true, std::memory_order_release);
  } catch (const std::exception & error) {
    release_handles();
    RCLCPP_ERROR(get_node()->get_logger(), "DaMiao head manager activation failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type HeadManagerController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load(std::memory_order_acquire)) {
    return controller_interface::return_type::OK;
  }
  try {
    const double phase_value = states_[0]->get_value();
    const double fault_value = states_[1]->get_value();
    const double motion_value = states_[3]->get_value();
    if (!std::isfinite(phase_value) || phase_value < 0.0 || phase_value > 6.0 ||
      std::trunc(phase_value) != phase_value || !exact_flag(fault_value) ||
      !exact_flag(motion_value))
    {
      return controller_interface::return_type::ERROR;
    }
    phase_.store(static_cast<std::uint8_t>(phase_value), std::memory_order_release);
    fault_latched_.store(fault_value == 1.0, std::memory_order_release);
    motion_allowed_.store(motion_value == 1.0, std::memory_order_release);
    handled_reset_generation_.store(
      exact_generation(states_[2]->get_value()), std::memory_order_release);
    commands_[0]->set_value(requested_enable_.load(std::memory_order_acquire) ? 1.0 : 0.0);
    commands_[1]->set_value(
      static_cast<double>(requested_reset_generation_.load(std::memory_order_acquire)));
    for (std::size_t motor{0U}; motor < 2U; ++motor) {
      const std::size_t state_offset = 4U + motor * 8U;
      for (std::size_t diagnostic{0U}; diagnostic < 4U; ++diagnostic) {
        const std::size_t state_index = state_offset + 3U + diagnostic;
        diagnostic_value_bits_[motor * 4U + diagnostic].store(
          encode_double(states_[state_index]->get_value()), std::memory_order_release);
      }
    }
    if (fault_latched_.load(std::memory_order_acquire) &&
      position_controller_active_.load(std::memory_order_acquire))
    {
      fault_stop_requested_.store(true, std::memory_order_release);
    }
  } catch (const std::exception &) {
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

HeadSnapshot HeadManagerController::snapshot() const noexcept
{
  return HeadSnapshot{
    static_cast<HeadPhase>(phase_.load(std::memory_order_acquire)),
    fault_latched_.load(std::memory_order_acquire),
    motion_allowed_.load(std::memory_order_acquire),
    handled_reset_generation_.load(std::memory_order_acquire)};
}

bool HeadManagerController::wait_for_result(
  Operation operation, std::uint64_t reset_generation) const
{
  const auto deadline = std::chrono::steady_clock::now() + service_timeout_;
  while (active_.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
    const HeadSnapshot current = snapshot();
    OperationResult result{OperationResult::pending};
    if (operation == Operation::enable) {
      result = assess_enable(current);
    } else if (operation == Operation::disable) {
      result = assess_disable(current);
    } else if (operation == Operation::reset) {
      result = assess_reset(current, reset_generation);
    }
    if (result != OperationResult::pending) {
      return result == OperationResult::success;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  }
  return false;
}

bool HeadManagerController::switch_position_controller(bool activate)
{
  if (!list_client_->wait_for_service(controller_switch_timeout_) ||
    !switch_client_->wait_for_service(controller_switch_timeout_))
  {
    return false;
  }
  auto list_future = list_client_->async_send_request(std::make_shared<ListControllers::Request>());
  if (list_future.wait_for(controller_switch_timeout_) != std::future_status::ready) {
    return false;
  }
  bool found = false;
  bool is_active = false;
  for (const auto & controller : list_future.get()->controller) {
    if (controller.name == position_controller_name_) {
      found = true;
      is_active = controller.state == "active";
      break;
    }
  }
  if (!found) {
    return false;
  }
  if (is_active == activate) {
    position_controller_active_.store(activate, std::memory_order_release);
    return true;
  }
  auto request = std::make_shared<SwitchController::Request>();
  if (activate) {
    request->activate_controllers.push_back(position_controller_name_);
  } else {
    request->deactivate_controllers.push_back(position_controller_name_);
  }
  request->strictness = SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout.sec = static_cast<std::int32_t>(controller_switch_timeout_.count() / 1000);
  request->timeout.nanosec = static_cast<std::uint32_t>(
    (controller_switch_timeout_.count() % 1000) * 1000000);
  auto switch_future = switch_client_->async_send_request(request);
  const bool success = switch_future.wait_for(controller_switch_timeout_) == std::future_status::ready &&
    switch_future.get()->ok;
  if (success) {
    position_controller_active_.store(activate, std::memory_order_release);
  }
  return success;
}

void HeadManagerController::handle_enable(
  const std::shared_ptr<EnableService::Request>,
  std::shared_ptr<EnableService::Response> response)
{
  Operation expected{Operation::none};
  if (!active_.load(std::memory_order_acquire) ||
    !operation_.compare_exchange_strong(expected, Operation::enable))
  {
    fill_response(*response, false, active_.load() ? "operation_in_progress" : "controller_inactive");
    return;
  }
  const auto release = [this]() {operation_.store(Operation::none, std::memory_order_release);};
  if (assess_enable(snapshot()) == OperationResult::success && switch_position_controller(true)) {
    fill_response(*response, true, "already_enabled");
    release();
    return;
  }
  if (snapshot().fault_latched) {
    fill_response(*response, false, "fault_requires_reset");
    release();
    return;
  }
  requested_enable_.store(true, std::memory_order_release);
  if (!wait_for_result(Operation::enable)) {
    requested_enable_.store(false, std::memory_order_release);
    fill_response(*response, false, snapshot().fault_latched ? "fault_detected" : "enable_timeout");
    release();
    return;
  }
  if (!switch_position_controller(true)) {
    requested_enable_.store(false, std::memory_order_release);
    static_cast<void>(wait_for_result(Operation::disable));
    fill_response(*response, false, "position_controller_activate_failed");
    release();
    return;
  }
  fill_response(*response, true, "success");
  release();
}

void HeadManagerController::handle_disable(
  const std::shared_ptr<EnableService::Request>,
  std::shared_ptr<EnableService::Response> response)
{
  Operation expected{Operation::none};
  if (!active_.load(std::memory_order_acquire) ||
    !operation_.compare_exchange_strong(expected, Operation::disable))
  {
    fill_response(*response, false, active_.load() ? "operation_in_progress" : "controller_inactive");
    return;
  }
  const bool controller_stopped = switch_position_controller(false);
  requested_enable_.store(false, std::memory_order_release);
  const bool disabled = wait_for_result(Operation::disable);
  fill_response(
    *response, controller_stopped && disabled,
    !controller_stopped ? "position_controller_deactivate_failed" :
    (snapshot().fault_latched ? "fault_requires_reset" : (disabled ? "success" : "disable_timeout")));
  operation_.store(Operation::none, std::memory_order_release);
}

void HeadManagerController::handle_reset(
  const std::shared_ptr<EnableService::Request>,
  std::shared_ptr<EnableService::Response> response)
{
  Operation expected{Operation::none};
  if (!active_.load(std::memory_order_acquire) ||
    !operation_.compare_exchange_strong(expected, Operation::reset))
  {
    fill_response(*response, false, active_.load() ? "operation_in_progress" : "controller_inactive");
    return;
  }
  if (!switch_position_controller(false)) {
    fill_response(*response, false, "position_controller_deactivate_failed");
    operation_.store(Operation::none, std::memory_order_release);
    return;
  }
  requested_enable_.store(false, std::memory_order_release);
  const auto generation = requested_reset_generation_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
  const bool reset = wait_for_result(Operation::reset, generation);
  fill_response(*response, reset, reset ? "success" : "fault_reset_timeout");
  operation_.store(Operation::none, std::memory_order_release);
}

void HeadManagerController::fill_response(
  EnableService::Response & response, bool ok, const std::string & stage) const
{
  response.ok = ok;
  response.failed_batch = -1;
  response.failed_joint = "";
  response.status_word = 0U;
  response.stage = stage;
}

void HeadManagerController::handle_fault_stop()
{
  if (fault_stop_requested_.exchange(false, std::memory_order_acq_rel)) {
    if (!switch_position_controller(false)) {
      fault_stop_requested_.store(true, std::memory_order_release);
    }
  }
}

void HeadManagerController::publish_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_node()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/robot/rt_control/damiao_head/summary";
  status.hardware_id = "alfa-v3-head";
  bool stale = false;
  for (std::size_t motor{0U}; motor < 2U; ++motor) {
    const double age = decode_double(
      diagnostic_value_bits_[motor * 4U + 3U].load(std::memory_order_acquire));
    stale = stale || !std::isfinite(age) || age < 0.0 || age > diagnostic_feedback_timeout_ms_;
  }
  const HeadSnapshot current = snapshot();
  const auto assessment = assess_diagnostics(current, stale);
  status.level = static_cast<std::uint8_t>(assessment.level);
  status.message = assessment.message;
  status.values.push_back(key_value("phase", phase_name(current.phase)));
  status.values.push_back(key_value("fault_latched", current.fault_latched ? "true" : "false"));
  status.values.push_back(key_value("motion_allowed", current.motion_allowed ? "true" : "false"));
  for (std::size_t motor{0U}; motor < 2U; ++motor) {
    const std::size_t offset = motor * 4U;
    const std::string prefix = joint_names_[motor] + ".";
    status.values.push_back(key_value(
      prefix + "fault_code", std::to_string(decode_double(
        diagnostic_value_bits_[offset].load(std::memory_order_acquire)))));
    status.values.push_back(key_value(
      prefix + "mos_temperature_c", std::to_string(decode_double(
        diagnostic_value_bits_[offset + 1U].load(std::memory_order_acquire)))));
    status.values.push_back(key_value(
      prefix + "motor_temperature_c", std::to_string(decode_double(
        diagnostic_value_bits_[offset + 2U].load(std::memory_order_acquire)))));
    status.values.push_back(key_value(
      prefix + "feedback_age_ms", std::to_string(decode_double(
        diagnostic_value_bits_[offset + 3U].load(std::memory_order_acquire)))));
  }
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

CallbackReturn HeadManagerController::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  requested_enable_.store(false, std::memory_order_release);
  if (commands_[0] != nullptr) {
    commands_[0]->set_value(0.0);
  }
  position_controller_active_.store(false, std::memory_order_release);
  release_handles();
  return CallbackReturn::SUCCESS;
}

CallbackReturn HeadManagerController::on_cleanup(const rclcpp_lifecycle::State &)
{
  enable_service_.reset();
  disable_service_.reset();
  reset_service_.reset();
  list_client_.reset();
  switch_client_.reset();
  diagnostics_timer_.reset();
  worker_timer_.reset();
  diagnostics_publisher_.reset();
  command_names_.clear();
  state_names_.clear();
  joint_names_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn HeadManagerController::on_error(const rclcpp_lifecycle::State & state)
{
  return on_deactivate(state);
}

void HeadManagerController::release_handles() noexcept
{
  commands_.fill(nullptr);
  states_.fill(nullptr);
  release_interfaces();
}

std::uint64_t HeadManagerController::encode_double(double value) noexcept
{
  std::uint64_t bits{0U};
  static_assert(sizeof(bits) == sizeof(value), "double snapshot requires 64-bit storage");
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

double HeadManagerController::decode_double(std::uint64_t bits) noexcept
{
  double value{0.0};
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace damiao_head_controller

PLUGINLIB_EXPORT_CLASS(
  damiao_head_controller::HeadManagerController,
  controller_interface::ControllerInterface)
