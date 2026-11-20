#include "ctrl_gravity_feedforward/gravity_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"
#include "hw_semantic_components/cia402_state.hpp"

namespace ctrl_gravity_feedforward
{
using CallbackReturn = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

CallbackReturn GravityController::on_init()
{
  for (const auto * key : {"robot_description", "base_link", "tip_link"}) {
    auto_declare<std::string>(key, "");
  }
  for (const auto * key : {"joints", "health_interfaces", "feedback_age_interfaces"}) {
    auto_declare<std::vector<std::string>>(key, {});
  }
  for (const auto * key :
       {"gravity_in_base", "max_effort_nm", "max_effort_rate_nm_s", "health_values"}) {
    auto_declare<std::vector<double>>(key, {});
  }
  auto_declare<double>("max_feedback_age_ms", 0.0);
  auto_declare<double>("scale", 0.0);
  auto_declare<bool>("gravity_in_root", false);
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityController::on_configure(const rclcpp_lifecycle::State & state)
{
  on_cleanup(state);
  try {
    const auto node = get_node();
    const auto gravity = node->get_parameter("gravity_in_base").as_double_array();
    const auto joints = node->get_parameter("joints").as_string_array();
    if (gravity.size() != 3 || joints.empty()) {
      throw std::invalid_argument("explicit joints and three gravity_in_base components required");
    }
    model_ = std::make_unique<GravityModel>(
      node->get_parameter("robot_description").as_string(),
      node->get_parameter("base_link").as_string(), node->get_parameter("tip_link").as_string(),
      std::array<double, 3>{gravity[0], gravity[1], gravity[2]},
      node->get_parameter("gravity_in_root").as_bool());
    scale_ = node->get_parameter("scale").as_double();
    if (!std::isfinite(scale_) || scale_ < 0.0 || scale_ > 1.0) {
      throw std::invalid_argument("scale must be in [0, 1]");
    }
    if (model_->joints() != joints) {
      throw std::invalid_argument("joints must match the URDF chain in base-to-tip order");
    }
    limits_ = node->get_parameter("max_effort_nm").as_double_array();
    rates_ = node->get_parameter("max_effort_rate_nm_s").as_double_array();
    max_feedback_age_ms_ = node->get_parameter("max_feedback_age_ms").as_double();
    if (
      limits_.size() != joints.size() || rates_.size() != joints.size() ||
      !std::isfinite(max_feedback_age_ms_) || max_feedback_age_ms_ <= 0.0) {
      throw std::invalid_argument(
        "explicit per-joint limits, rates and feedback age limit required");
    }
    for (std::size_t index = 0; index < joints.size(); ++index) {
      if (
        !std::isfinite(limits_[index]) || limits_[index] <= 0.0 || !std::isfinite(rates_[index]) ||
        rates_[index] <= 0.0) {
        throw std::invalid_argument("effort limits and rates must be finite and positive");
      }
      command_names_.push_back(joints[index] + "/effort");
      state_names_.push_back(joints[index] + "/position");
      state_names_.push_back(joints[index] + "/status_word");
      state_names_.push_back(joints[index] + "/gravity_command_epoch");
    }
    const auto ages = node->get_parameter("feedback_age_interfaces").as_string_array();
    const auto health = node->get_parameter("health_interfaces").as_string_array();
    health_values_ = node->get_parameter("health_values").as_double_array();
    if (ages.empty() || health.empty() || health.size() != health_values_.size()) {
      throw std::invalid_argument(
        "explicit age and health interfaces with expected values required");
    }
    age_count_ = ages.size();
    for (const auto & age : ages) {
      if (age.empty()) {
        throw std::invalid_argument("empty feedback age interface");
      }
      state_names_.push_back(age);
    }
    for (std::size_t index = 0; index < health.size(); ++index) {
      if (health[index].empty() || !std::isfinite(health_values_[index])) {
        throw std::invalid_argument("invalid health interface or expected value");
      }
      state_names_.push_back(health[index]);
    }
    if (
      std::set<std::string>(state_names_.begin(), state_names_.end()).size() !=
      state_names_.size()) {
      throw std::invalid_argument("state interfaces must be unique");
    }
    outputs_.resize(joints.size(), nullptr);
    inputs_.resize(state_names_.size(), nullptr);
    positions_.resize(joints.size(), 0.0);
    last_effort_.resize(joints.size(), 0.0);
    epochs_.resize(joints.size(), 0.0);
  } catch (const std::exception & error) {
    on_cleanup(state);
    RCLCPP_ERROR(get_node()->get_logger(), "Gravity configuration rejected: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GravityController::command_interface_configuration()
  const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, command_names_};
}

controller_interface::InterfaceConfiguration GravityController::state_interface_configuration()
  const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, state_names_};
}

CallbackReturn GravityController::on_activate(const rclcpp_lifecycle::State &)
{
  zero_commands();
  active_ = false;
  faulted_ = false;
  std::fill(inputs_.begin(), inputs_.end(), nullptr);
  std::fill(outputs_.begin(), outputs_.end(), nullptr);
  if (
    !model_ || command_interfaces_.size() != command_names_.size() ||
    state_interfaces_.size() != state_names_.size()) {
    return CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < command_names_.size(); ++index) {
    for (auto & interface : command_interfaces_) {
      if (interface.get_name() == command_names_[index]) {
        if (outputs_[index]) {
          return CallbackReturn::ERROR;
        }
        outputs_[index] = &interface;
      }
    }
    if (!outputs_[index]) {
      return CallbackReturn::ERROR;
    }
  }
  for (std::size_t index = 0; index < state_names_.size(); ++index) {
    for (auto & interface : state_interfaces_) {
      if (interface.get_name() == state_names_[index]) {
        if (inputs_[index]) {
          return CallbackReturn::ERROR;
        }
        inputs_[index] = &interface;
      }
    }
    if (!inputs_[index]) {
      return CallbackReturn::ERROR;
    }
  }
  if (!feedback_ready()) {
    return CallbackReturn::ERROR;
  }
  active_ = true;
  return CallbackReturn::SUCCESS;
}

bool GravityController::feedback_ready() const noexcept
{
  using State = hw_semantic_components::Cia402State;
  const auto count = positions_.size();
  for (std::size_t index = 0; index < count; ++index) {
    const double position = inputs_[3 * index]->get_value();
    const double status = inputs_[3 * index + 1]->get_value();
    const double epoch = inputs_[3 * index + 2]->get_value();
    if (
      !std::isfinite(epoch) || epoch < 0.0 || std::trunc(epoch) != epoch ||
      !std::isfinite(position) || !std::isfinite(status) || status < 0.0 || status > 65535.0 ||
      std::trunc(status) != status ||
      hw_semantic_components::decode_cia402_state(static_cast<std::uint16_t>(status)) !=
        State::kOperationEnabled) {
      return false;
    }
  }
  for (std::size_t index = 0; index < age_count_; ++index) {
    const double age = inputs_[3 * count + index]->get_value();
    if (!std::isfinite(age) || age < 0.0 || age > max_feedback_age_ms_) {
      return false;
    }
  }
  for (std::size_t index = 0; index < health_values_.size(); ++index) {
    if (inputs_[3 * count + age_count_ + index]->get_value() != health_values_[index]) {
      return false;
    }
  }
  return true;
}

Return GravityController::update(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_) {
    return Return::OK;
  }
  const double seconds = period.seconds();
  if (faulted_ || !std::isfinite(seconds) || seconds <= 0.0 || !feedback_ready()) {
    faulted_ = true;
    zero_commands();
    return Return::ERROR;
  }
  for (std::size_t index = 0; index < positions_.size(); ++index) {
    positions_[index] = inputs_[3 * index]->get_value();
    const double epoch = inputs_[3 * index + 2]->get_value();
    if (epoch != epochs_[index]) {
      last_effort_[index] = 0.0;
      epochs_[index] = epoch;
    }
  }
  if (!model_->compute(positions_)) {
    faulted_ = true;
    zero_commands();
    return Return::ERROR;
  }
  // Stage all commands before publishing any axis; one invalid result zeros the group.
  for (std::size_t index = 0; index < positions_.size(); ++index) {
    const double target =
      std::clamp(scale_ * model_->torques()[index], -limits_[index], limits_[index]);
    const double step = rates_[index] * seconds;
    if (!std::isfinite(step)) {
      faulted_ = true;
      zero_commands();
      return Return::ERROR;
    }
    last_effort_[index] += std::clamp(target - last_effort_[index], -step, step);
  }
  for (std::size_t index = 0; index < positions_.size(); ++index) {
    outputs_[index]->set_value(last_effort_[index]);
  }
  return Return::OK;
}

void GravityController::zero_commands() noexcept
{
  for (auto & interface : command_interfaces_) {
    interface.set_value(0.0);
  }
  std::fill(last_effort_.begin(), last_effort_.end(), 0.0);
}

CallbackReturn GravityController::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  zero_commands();
  std::fill(inputs_.begin(), inputs_.end(), nullptr);
  std::fill(outputs_.begin(), outputs_.end(), nullptr);
  release_interfaces();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityController::on_cleanup(const rclcpp_lifecycle::State & state)
{
  on_deactivate(state);
  model_.reset();
  command_names_.clear();
  state_names_.clear();
  positions_.clear();
  limits_.clear();
  rates_.clear();
  health_values_.clear();
  inputs_.clear();
  outputs_.clear();
  last_effort_.clear();
  epochs_.clear();
  faulted_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityController::on_error(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

CallbackReturn GravityController::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

}  // namespace ctrl_gravity_feedforward

PLUGINLIB_EXPORT_CLASS(
  ctrl_gravity_feedforward::GravityController, controller_interface::ControllerInterface)
