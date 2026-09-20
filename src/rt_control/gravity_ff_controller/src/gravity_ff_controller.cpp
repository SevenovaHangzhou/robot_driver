#include "gravity_ff_controller/gravity_ff_controller.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <limits>
#include <openssl/evp.h>
#include <set>
#include <sstream>

namespace gravity_ff_controller
{
namespace
{
std::string sha256(const std::string & data)
{
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int size = 0U;
  auto context = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>(
    EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context ||
    EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1 ||
    EVP_DigestUpdate(context.get(), data.data(), data.size()) != 1 ||
    EVP_DigestFinal_ex(context.get(), digest.data(), &size) != 1)
  {
    return {};
  }
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (unsigned int i = 0U; i < size; ++i) {
    result << std::setw(2) << static_cast<int>(digest[i]);
  }
  return result.str();
}

template<typename Interface>
Interface * find_interface(
  std::vector<Interface> & interfaces,
  const std::string & full_name)
{
  auto match =
    std::find_if(
    interfaces.begin(), interfaces.end(), [&](auto & interface) {
      return interface.get_name() == full_name;
    });
  return match == interfaces.end() ? nullptr : &*match;
}

void set_number(std::string & target, const double value) noexcept
{
  char buffer[64];
  const int count = std::snprintf(buffer, sizeof(buffer), "%.12g", value);
  target.assign(buffer, count > 0 ? static_cast<std::size_t>(count) : 0U);
}
} // namespace

controller_interface::CallbackReturn GravityFeedforwardController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("mode", "shadow");
    auto_declare<std::vector<double>>("scale", {});
    auto_declare<std::vector<double>>("max_effort_nm", {});
    auto_declare<std::vector<double>>("max_slew_nm_per_s", {});
    auto_declare<double>("fault_slew_nm_per_s", 1000.0);
    auto_declare<std::string>("position_interface", "position");
    auto_declare<std::string>("status_word_interface", "status_word");
    auto_declare<std::string>(
      "torque_actual_interface",
      "torque_actual_permille");
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::vector<std::string>>("locked_joints", {});
    auto_declare<std::vector<double>>("locked_positions", {});
    auto_declare<std::vector<double>>("gravity", {0.0, 0.0, -9.81});
    auto_declare<std::vector<double>>("input_inertia_g_mm2", {});
    auto_declare<std::vector<double>>("reduction_ratio", {});
    auto_declare<std::string>("payload_frame", "");
    auto_declare<double>("payload.mass_kg", 0.0);
    auto_declare<std::vector<double>>("payload.com_xyz", {0.0, 0.0, 0.0});
    auto_declare<std::vector<double>>(
      "payload.inertia", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    auto_declare<bool>("model_validation.verified", false);
    auto_declare<std::string>("model_validation.urdf_sha256", "");
    auto_declare<std::string>("model_validation.source", "");
    auto_declare<std::vector<bool>>("effort_calibration.verified", {});
    auto_declare<std::vector<double>>(
      "effort_calibration.permille_per_newton_metre", {});
    auto_declare<std::vector<std::string>>("effort_calibration.source", {});
    auto_declare<std::string>("diagnostic_topic", "~/state");
    auto_declare<double>("diagnostic_publish_rate", 10.0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "parameter declaration failed: %s",
      error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool GravityFeedforwardController::configure_model()
{
  const auto urdf = get_node()->get_parameter("robot_description").as_string();
  const auto expected_hash =
    get_node()->get_parameter("model_validation.urdf_sha256").as_string();
  const auto model_source =
    get_node()->get_parameter("model_validation.source").as_string();
  const auto computed_hash = sha256(urdf);
  model_verified_ =
    get_node()->get_parameter("model_validation.verified").as_bool() &&
    !model_source.empty() && model_source != "TBD" &&
    expected_hash.size() == 64U &&
    computed_hash == expected_hash;
  if (active_mode_ && !model_verified_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "active mode requires verified model source and matching "
      "original URDF SHA-256");
    return false;
  }
  rt_arm_dynamics::Config config;
  config.urdf = urdf;
  config.joints = joints_;
  const auto locked =
    get_node()->get_parameter("locked_joints").as_string_array();
  const auto positions =
    get_node()->get_parameter("locked_positions").as_double_array();
  if (locked.size() != positions.size()) {
    return false;
  }
  for (std::size_t i = 0U; i < locked.size(); ++i) {
    config.locked_positions.emplace(locked[i], positions[i]);
  }
  const auto gravity = get_node()->get_parameter("gravity").as_double_array();
  if (gravity.size() != 3U) {
    return false;
  }
  config.gravity = Eigen::Vector3d(gravity[0], gravity[1], gravity[2]);
  config.input_inertia_g_mm2 =
    get_node()->get_parameter("input_inertia_g_mm2").as_double_array();
  config.reduction_ratio =
    get_node()->get_parameter("reduction_ratio").as_double_array();
  config.payload_frame = get_node()->get_parameter("payload_frame").as_string();
  std::string reason;
  if (dynamics_.initialize(config, reason) != rt_arm_dynamics::Result::kOk) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "arm model initialization failed: %s", reason.c_str());
    return false;
  }
  const auto com =
    get_node()->get_parameter("payload.com_xyz").as_double_array();
  const auto inertia =
    get_node()->get_parameter("payload.inertia").as_double_array();
  if (com.size() != 3U || inertia.size() != 9U) {
    return false;
  }
  Eigen::Matrix3d tensor;
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index column = 0; column < 3; ++column) {
      tensor(row, column) = inertia[static_cast<std::size_t>(row * 3 + column)];
    }
  }
  return dynamics_.setPayload(
    get_node()->get_parameter("payload.mass_kg").as_double(),
    Eigen::Vector3d(com[0], com[1], com[2]),
    tensor) == rt_arm_dynamics::Result::kOk;
}

bool GravityFeedforwardController::configure_policy()
{
  const auto mode = get_node()->get_parameter("mode").as_string();
  if (mode != "shadow" && mode != "active") {
    return false;
  }
  active_mode_ = mode == "active";
  auto scales = get_node()->get_parameter("scale").as_double_array();
  if (scales.empty()) {
    scales.assign(joints_.size(), 0.0);
  }
  if (scales.size() != joints_.size()) {
    return false;
  }
  for (std::size_t i = 0U; i < joints_.size(); ++i) {
    if (!std::isfinite(scales[i]) || scales[i] < 0.0 || scales[i] > 1.0) {
      return false;
    }
    scale_[i].store(scales[i], std::memory_order_relaxed);
  }
  if (active_mode_) {
    Limits limits{
      get_node()->get_parameter("max_effort_nm").as_double_array(),
      get_node()->get_parameter("max_slew_nm_per_s").as_double_array(),
      get_node()->get_parameter("fault_slew_nm_per_s").as_double()};
    if (!core_.configure(limits)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "explicit positive max_effort_nm and max_slew_nm_per_s are "
        "required for every joint in active mode");
      return false;
    }
  }
  const auto verified =
    get_node()->get_parameter("effort_calibration.verified").as_bool_array();
  const auto coefficients =
    get_node()
    ->get_parameter("effort_calibration.permille_per_newton_metre")
    .as_double_array();
  const auto sources =
    get_node()->get_parameter("effort_calibration.source").as_string_array();
  calibration_verified_ = verified.size() == joints_.size() &&
    coefficients.size() == joints_.size() &&
    sources.size() == joints_.size();
  for (std::size_t i = 0U; calibration_verified_ && i < joints_.size(); ++i) {
    calibration_verified_ = verified[i] && std::isfinite(coefficients[i]) &&
      coefficients[i] > 0.0 && !sources[i].empty() && sources[i] != "TBD";
  }
  if (active_mode_ && !calibration_verified_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "active mode requires verified positive effort calibration "
      "for every joint");
    return false;
  }
  const double rate =
    get_node()->get_parameter("diagnostic_publish_rate").as_double();
  if (!std::isfinite(rate) || rate <= 0.0 || rate > 100.0) {
    return false;
  }
  publish_period_seconds_ = 1.0 / rate;
  return true;
}

controller_interface::CallbackReturn
GravityFeedforwardController::on_configure(const rclcpp_lifecycle::State &)
{
  clear_resources();
  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.empty() || joints_.size() > kMaximumAxes ||
    std::set<std::string>(joints_.begin(), joints_.end()).size() !=
    joints_.size() ||
    !configure_policy() || !configure_model())
  {
    clear_resources();
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto position_name =
    get_node()->get_parameter("position_interface").as_string();
  const auto status_name =
    get_node()->get_parameter("status_word_interface").as_string();
  const auto torque_name =
    get_node()->get_parameter("torque_actual_interface").as_string();
  if (position_name.empty() || status_name.empty() ||
    (!active_mode_ && torque_name.empty()))
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  torque_state_enabled_ = !torque_name.empty();
  for (const auto & joint : joints_) {
    state_names_.insert(
      state_names_.end(), {joint + "/" + position_name, joint + "/" + status_name});
    if (torque_state_enabled_) {
      state_names_.push_back(joint + "/" + torque_name);
    }
    if (active_mode_) {
      command_names_.push_back(joint + "/effort");
    }
  }
  const auto count = static_cast<Eigen::Index>(joints_.size());
  q_ = Eigen::VectorXd::Zero(count);
  gravity_nm_ = Eigen::VectorXd::Zero(count);
  gravity_values_.assign(joints_.size(), 0.0);
  status_values_.assign(joints_.size(), 0.0);
  scale_values_.assign(joints_.size(), 0.0);
  torque_raw_values_.assign(joints_.size(), 0.0);
  publisher_ =
    get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    get_node()->get_parameter("diagnostic_topic").as_string(),
    rclcpp::QoS(1));
  realtime_publisher_ = std::make_unique<
    realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
    publisher_);
  auto & diagnostic = realtime_publisher_->msg_.status.emplace_back();
  diagnostic.name = get_node()->get_name();
  diagnostic.hardware_id = "gravity_feedforward";
  diagnostic.message.reserve(64U);
  diagnostic.values.resize(3U + joints_.size() * 3U);
  diagnostic.values[0].key = "model_verified";
  diagnostic.values[1].key = "effort_calibration_verified";
  diagnostic.values[2].key = "fault_latched";
  for (std::size_t i = 0U; i < joints_.size(); ++i) {
    diagnostic.values[3U + i * 3U].key = "gravity_nm." + joints_[i];
    diagnostic.values[4U + i * 3U].key = "torque_actual_permille." + joints_[i];
    diagnostic.values[5U + i * 3U].key = "output_nm." + joints_[i];
  }
  for (auto & value : diagnostic.values) {
    value.value.reserve(64U);
  }
  parameter_callback_ = get_node()->add_on_set_parameters_callback(
    std::bind(
      &GravityFeedforwardController::validate_parameters, this,
      std::placeholders::_1));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GravityFeedforwardController::command_interface_configuration() const
{
  return {active_mode_ ?
    controller_interface::interface_configuration_type::INDIVIDUAL :
    controller_interface::interface_configuration_type::NONE,
    command_names_};
}
controller_interface::InterfaceConfiguration
GravityFeedforwardController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    state_names_};
}

bool GravityFeedforwardController::bind_interfaces()
{
  const std::size_t state_stride = torque_state_enabled_ ? 3U : 2U;
  for (std::size_t i = 0U; i < joints_.size(); ++i) {
    position_.push_back(
      find_interface(state_interfaces_, state_names_[i * state_stride]));
    status_.push_back(
      find_interface(state_interfaces_, state_names_[i * state_stride + 1U]));
    torque_raw_.push_back(
      torque_state_enabled_ ?
      find_interface(state_interfaces_, state_names_[i * state_stride + 2U]) :
      nullptr);
    if (active_mode_) {
      effort_.push_back(find_interface(command_interfaces_, command_names_[i]));
    }
  }
  return std::none_of(
    position_.begin(), position_.end(),
    [](auto * item) {return item == nullptr;}) &&
         std::none_of(
    status_.begin(), status_.end(),
    [](auto * item) {return item == nullptr;}) &&
         std::none_of(
    torque_raw_.begin(), torque_raw_.end(),
    [this](auto * item) {return torque_state_enabled_ && item == nullptr;}) &&
         std::none_of(
    effort_.begin(), effort_.end(),
    [](auto * item) {return item == nullptr;});
}

controller_interface::CallbackReturn
GravityFeedforwardController::on_activate(const rclcpp_lifecycle::State &)
{
  if (!bind_interfaces()) {
    clear_resources();
    return controller_interface::CallbackReturn::ERROR;
  }
  if (active_mode_) {
    core_.activate();
  }
  shadow_error_ = false;
  publish_elapsed_seconds_ = 0.0;
  if (active_mode_) {
    for (auto * interface : effort_) {
      interface->set_value(0.0);
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
GravityFeedforwardController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  for (std::size_t i = 0U; i < joints_.size(); ++i) {
    q_[static_cast<Eigen::Index>(i)] = position_[i]->get_value();
    status_values_[i] = status_[i]->get_value();
    torque_raw_values_[i] = torque_state_enabled_ ? torque_raw_[i]->get_value() :
      std::numeric_limits<double>::quiet_NaN();
    scale_values_[i] = scale_[i].load(std::memory_order_relaxed);
  }
  const bool input_valid =
    q_.allFinite() &&
    dynamics_.gravity(q_, gravity_nm_) == rt_arm_dynamics::Result::kOk;
  if (input_valid) {
    for (std::size_t i = 0U; i < joints_.size(); ++i) {
      gravity_values_[i] = gravity_nm_[static_cast<Eigen::Index>(i)];
    }
  }
  const double seconds = period.seconds();
  StepStatus result = StepStatus::kOk;
  if (active_mode_) {
    result = core_.step(
      gravity_values_, scale_values_, status_values_, input_valid, seconds);
    for (std::size_t i = 0U; i < effort_.size(); ++i) {
      effort_[i]->set_value(core_.output_nm()[i]);
    }
  } else {
    shadow_error_ = !input_valid;
    for (const double raw_status : status_values_) {
      const auto state = ControlCore::decode(raw_status);
      shadow_error_ = shadow_error_ ||
        state == DriveState::kUnknown || state == DriveState::kFault;
      if (state == DriveState::kFaultReaction) {
        result = StepStatus::kFaultReactionHold;
      }
    }
    if (shadow_error_) {
      result = StepStatus::kInvalidInputLatched;
    }
  }
  publish_elapsed_seconds_ +=
    std::isfinite(seconds) && seconds > 0.0 ? seconds : 0.0;
  if (publish_elapsed_seconds_ >= publish_period_seconds_) {
    publish_elapsed_seconds_ = 0.0;
    publish_diagnostics(time, result);
  }
  return controller_interface::return_type::OK;
}

void GravityFeedforwardController::publish_diagnostics(
  const rclcpp::Time & time, const StepStatus result)
{
  if (!realtime_publisher_ || !realtime_publisher_->trylock()) {
    return;
  }
  auto & message = realtime_publisher_->msg_;
  message.header.stamp = time;
  auto & diagnostic = message.status[0];
  diagnostic.level = (active_mode_ ? core_.fault_latched() : shadow_error_) ?
    diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    (!model_verified_ || !calibration_verified_ ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK);
  diagnostic.message =
    result == StepStatus::kFaultReactionHold ?
    "fault_reaction_hold" :
    (result == StepStatus::kDisableTransitionHold ?
    "disable_transition_hold" :
    (active_mode_ && core_.fault_latched() ?
    "fault_latched_reactivate_required" :
    (shadow_error_ ? "invalid_shadow_input" : "operational")));
  diagnostic.values[0].value = model_verified_ ? "true" : "false";
  diagnostic.values[1].value = calibration_verified_ ? "true" : "false";
  diagnostic.values[2].value = active_mode_ && core_.fault_latched() ? "true" : "false";
  for (std::size_t i = 0U; i < joints_.size(); ++i) {
    set_number(
      diagnostic.values[3U + i * 3U].value,
      shadow_error_ && !active_mode_ ?
      std::numeric_limits<double>::quiet_NaN() : gravity_values_[i]);
    if (torque_state_enabled_) {
      set_number(diagnostic.values[4U + i * 3U].value, torque_raw_values_[i]);
    } else {
      diagnostic.values[4U + i * 3U].value = "unavailable";
    }
    set_number(
      diagnostic.values[5U + i * 3U].value,
      active_mode_ ? core_.output_nm()[i] : 0.0);
  }
  realtime_publisher_->unlockAndPublish();
}

rcl_interfaces::msg::SetParametersResult
GravityFeedforwardController::validate_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool has_scale = false;
  std::vector<double> candidate;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() != "scale") {
      result.successful = false;
      result.reason = "only scale is mutable after configure";
      return result;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      result.successful = false;
      result.reason = "scale must be a double array";
      return result;
    }
    candidate = parameter.as_double_array();
    if (candidate.size() != joints_.size()) {
      result.successful = false;
      result.reason = "scale length must equal joints";
      return result;
    }
    for (const double value : candidate) {
      if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
        result.successful = false;
        result.reason = "scale values must be finite in [0,1]";
        return result;
      }
    }
    has_scale = true;
  }
  if (has_scale) {
    for (std::size_t i = 0U; i < candidate.size(); ++i) {
      scale_[i].store(candidate[i], std::memory_order_relaxed);
    }
  }
  return result;
}

controller_interface::CallbackReturn
GravityFeedforwardController::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (active_mode_) {
    for (auto * interface : effort_) {
      if (interface) {
        interface->set_value(0.0);
      }
    }
  }
  position_.clear();
  status_.clear();
  torque_raw_.clear();
  effort_.clear();
  core_.force_zero();
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn
GravityFeedforwardController::on_cleanup(const rclcpp_lifecycle::State &)
{
  clear_resources();
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn
GravityFeedforwardController::on_error(const rclcpp_lifecycle::State & state)
{
  if (active_mode_) {
    for (auto * interface : effort_) {
      if (interface) {
        interface->set_value(0.0);
      }
    }
  }
  return on_cleanup(state);
}

void GravityFeedforwardController::clear_resources()
{
  position_.clear();
  status_.clear();
  torque_raw_.clear();
  effort_.clear();
  state_names_.clear();
  command_names_.clear();
  joints_.clear();
  parameter_callback_.reset();
  realtime_publisher_.reset();
  publisher_.reset();
}
} // namespace gravity_ff_controller

PLUGINLIB_EXPORT_CLASS(
  gravity_ff_controller::GravityFeedforwardController,
  controller_interface::ControllerInterface)
