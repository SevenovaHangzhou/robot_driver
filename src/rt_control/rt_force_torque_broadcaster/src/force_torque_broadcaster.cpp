#include "rt_force_torque_broadcaster/force_torque_broadcaster.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robot_interfaces_qos/profiles.hpp"

namespace rt_force_torque_broadcaster
{
namespace
{

using CallbackReturn = controller_interface::CallbackReturn;
constexpr std::size_t kAxisCount{6U};

bool configured_name(const std::string & value)
{
  return !value.empty() && value.find("TBD") == std::string::npos;
}

std::int32_t checked_int32(const std::int64_t value, const char * name)
{
  if (
    value < static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min()) ||
    value > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()))
  {
    throw std::invalid_argument(std::string{name} + " must fit int32");
  }
  return static_cast<std::int32_t>(value);
}

}  // namespace

CallbackReturn ForceTorqueBroadcaster::on_init()
{
  for (const auto * name : {
      "sensor_name", "frame_id", "validity_policy", "startup_id", "snapshot_source",
      "wrench_topic", "raw_topic", "calibration_topic", "diagnostic_name",
      "link_interface", "al_state_interface"})
  {
    auto_declare<std::string>(name, "");
  }
  for (const auto * name : {"value_interfaces", "auxiliary_interfaces"}) {
    auto_declare<std::vector<std::string>>(name, {});
  }
  auto_declare<std::vector<double>>("scale_factors", {});
  auto_declare<std::vector<std::int64_t>>("decimals", {});
  auto_declare<std::vector<std::int64_t>>("unit_codes", {});
  auto_declare<std::int64_t>("minimum_auxiliary_value", 0);
  auto_declare<std::int64_t>("maximum_auxiliary_value", 0);
  auto_declare<bool>("calibration_valid", false);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ForceTorqueBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  if (active_.load(std::memory_order_acquire)) {
    return CallbackReturn::ERROR;
  }

  try {
    clear_runtime_resources();
    sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
    frame_id_ = get_node()->get_parameter("frame_id").as_string();
    wrench_topic_ = get_node()->get_parameter("wrench_topic").as_string();
    raw_topic_ = get_node()->get_parameter("raw_topic").as_string();
    calibration_topic_ = get_node()->get_parameter("calibration_topic").as_string();
    diagnostic_name_ = get_node()->get_parameter("diagnostic_name").as_string();
    link_interface_name_ = get_node()->get_parameter("link_interface").as_string();
    al_state_interface_name_ = get_node()->get_parameter("al_state_interface").as_string();
    validity_policy_ = get_node()->get_parameter("validity_policy").as_string();
    snapshot_source_ = get_node()->get_parameter("snapshot_source").as_string();
    const auto startup_id = get_node()->get_parameter("startup_id").as_string();
    const std::array<std::reference_wrapper<const std::string>, 11> required_strings{
      sensor_name_, frame_id_, wrench_topic_, raw_topic_, calibration_topic_,
      diagnostic_name_, link_interface_name_, al_state_interface_name_, validity_policy_,
      snapshot_source_, startup_id};
    for (const auto & value : required_strings) {
      if (!configured_name(value.get())) {
        throw std::invalid_argument("force/torque string parameters must be explicit");
      }
    }

    const auto value_names = get_node()->get_parameter("value_interfaces").as_string_array();
    const auto auxiliary_names =
      get_node()->get_parameter("auxiliary_interfaces").as_string_array();
    if (value_names.size() != kAxisCount) {
      throw std::invalid_argument("force/torque value_interfaces must contain six names");
    }
    std::array<std::string, kAxisCount> value_interfaces;
    std::copy(value_names.begin(), value_names.end(), value_interfaces.begin());
    auto candidate_sensor =
      std::make_unique<rt_control_semantic_components::ForceTorqueSensor>(
      std::move(value_interfaces), auxiliary_names);

    if (
      link_interface_name_ == al_state_interface_name_ ||
      std::find(
        candidate_sensor->get_state_interface_names().begin(),
        candidate_sensor->get_state_interface_names().end(), link_interface_name_) !=
      candidate_sensor->get_state_interface_names().end() ||
      std::find(
        candidate_sensor->get_state_interface_names().begin(),
        candidate_sensor->get_state_interface_names().end(), al_state_interface_name_) !=
      candidate_sensor->get_state_interface_names().end())
    {
      throw std::invalid_argument("force/torque link and AL interfaces must be unique");
    }

    const auto scales = get_node()->get_parameter("scale_factors").as_double_array();
    if (scales.size() != kAxisCount) {
      throw std::invalid_argument("force/torque scale_factors must contain six values");
    }
    ForceTorqueProcessorConfig processor_config;
    std::copy(scales.begin(), scales.end(), processor_config.scale_factors.begin());
    if (validity_policy_ == "all_exact_in_range") {
      processor_config.auxiliary_policy = AuxiliaryPolicy::kAllExactIntegersInRange;
    } else if (validity_policy_ == "none") {
      processor_config.auxiliary_policy = AuxiliaryPolicy::kNone;
    } else {
      throw std::invalid_argument("unsupported force/torque validity_policy");
    }
    processor_config.auxiliary_count = auxiliary_names.size();
    minimum_auxiliary_value_ =
      get_node()->get_parameter("minimum_auxiliary_value").as_int();
    maximum_auxiliary_value_ =
      get_node()->get_parameter("maximum_auxiliary_value").as_int();
    processor_config.minimum_auxiliary_value = checked_int32(
      minimum_auxiliary_value_, "minimum_auxiliary_value");
    processor_config.maximum_auxiliary_value = checked_int32(
      maximum_auxiliary_value_, "maximum_auxiliary_value");

    const bool requested_calibration =
      get_node()->get_parameter("calibration_valid").as_bool();
    decimals_ = get_node()->get_parameter("decimals").as_integer_array();
    unit_codes_ = get_node()->get_parameter("unit_codes").as_integer_array();
    if (requested_calibration) {
      if (snapshot_source_ != "preop_sdo" && snapshot_source_ != "fixed_protocol") {
        throw std::invalid_argument("valid calibration requires an approved snapshot source");
      }
      if (decimals_.size() != kAxisCount || unit_codes_.size() != kAxisCount) {
        throw std::invalid_argument("valid calibration requires six decimals and unit codes");
      }
      for (std::size_t index = 0U; index < kAxisCount; ++index) {
        if (
          decimals_[index] < 0 || decimals_[index] > 10 ||
          unit_codes_[index] != (index < 3U ? 5 : 7))
        {
          throw std::invalid_argument("invalid force/torque engineering-unit metadata");
        }
        const double expected_scale = std::pow(10.0, -static_cast<double>(decimals_[index]));
        if (
          !std::isfinite(scales[index]) ||
          std::abs(scales[index] - expected_scale) >
          std::numeric_limits<double>::epsilon() * expected_scale * 8.0)
        {
          throw std::invalid_argument("scale_factors do not match calibration decimals");
        }
      }
    } else if (
      (!decimals_.empty() && decimals_.size() != kAxisCount) ||
      (!unit_codes_.empty() && unit_codes_.size() != kAxisCount))
    {
      throw std::invalid_argument("partial force/torque metadata is not allowed");
    }

    if (startup_id_ != startup_id) {
      startup_id_ = startup_id;
      startup_invalidated_.store(false, std::memory_order_release);
    }
    calibration_valid_ = requested_calibration &&
      !startup_invalidated_.load(std::memory_order_acquire);
    processor_config.calibration_valid = calibration_valid_;
    auto candidate_processor = std::make_unique<ForceTorqueProcessor>(processor_config);

    state_interface_names_ = candidate_sensor->get_state_interface_names();
    state_interface_names_.push_back(link_interface_name_);
    state_interface_names_.push_back(al_state_interface_name_);
    sensor_ = std::move(candidate_sensor);
    processor_ = std::move(candidate_processor);

    wrench_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      wrench_topic_, robot_interfaces_qos::fast_state());
    realtime_wrench_publisher_ = std::make_unique<
      realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
      wrench_publisher_);
    realtime_wrench_publisher_->msg_.header.frame_id = frame_id_;
    raw_publisher_ = get_node()->create_publisher<std_msgs::msg::Int32MultiArray>(
      raw_topic_, robot_interfaces_qos::fast_state());
    realtime_raw_publisher_ = std::make_unique<
      realtime_tools::RealtimePublisher<std_msgs::msg::Int32MultiArray>>(
      raw_publisher_);
    realtime_raw_publisher_->msg_.data.resize(kAxisCount);
    calibration_publisher_ =
      get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      calibration_topic_, robot_interfaces_qos::latched());
    calibration_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(100), [this]() {publish_calibration();});
    diagnostic_generation_.fetch_add(1U, std::memory_order_acq_rel);
    published_diagnostic_generation_ = 0U;
  } catch (const std::exception & error) {
    clear_runtime_resources();
    RCLCPP_ERROR(
      get_node()->get_logger(), "Force/torque configuration rejected: %s", error.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ForceTorqueBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
ForceTorqueBroadcaster::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    state_interface_names_};
}

CallbackReturn ForceTorqueBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  if (active_.load(std::memory_order_acquire) || sensor_ == nullptr || processor_ == nullptr) {
    return CallbackReturn::ERROR;
  }
  if (!sensor_->assign_loaned_state_interfaces(state_interfaces_)) {
    return CallbackReturn::ERROR;
  }

  for (auto & interface : state_interfaces_) {
    if (interface.get_name() == link_interface_name_) {
      if (link_interface_ != nullptr) {
        sensor_->release_interfaces();
        link_interface_ = nullptr;
        return CallbackReturn::ERROR;
      }
      link_interface_ = &interface;
    }
    if (interface.get_name() == al_state_interface_name_) {
      if (al_state_interface_ != nullptr) {
        sensor_->release_interfaces();
        al_state_interface_ = nullptr;
        return CallbackReturn::ERROR;
      }
      al_state_interface_ = &interface;
    }
  }
  if (link_interface_ == nullptr || al_state_interface_ == nullptr) {
    sensor_->release_interfaces();
    link_interface_ = nullptr;
    al_state_interface_ = nullptr;
    return CallbackReturn::ERROR;
  }

  active_.store(true, std::memory_order_release);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ForceTorqueBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (
    !active_.load(std::memory_order_acquire) || sensor_ == nullptr ||
    processor_ == nullptr || link_interface_ == nullptr || al_state_interface_ == nullptr)
  {
    return controller_interface::return_type::OK;
  }

  const auto sample = sensor_->read();
  if (!sample.has_value()) {
    return controller_interface::return_type::ERROR;
  }
  const auto outcome = processor_->process(
    *sample, link_interface_->get_value(), al_state_interface_->get_value());
  if (outcome.invalidated_this_cycle) {
    startup_invalidated_.store(true, std::memory_order_release);
    diagnostic_generation_.fetch_add(1U, std::memory_order_acq_rel);
  }
  if (outcome.raw_valid && realtime_raw_publisher_->trylock()) {
    for (std::size_t index = 0U; index < kAxisCount; ++index) {
      realtime_raw_publisher_->msg_.data[index] = outcome.raw_values[index];
    }
    realtime_raw_publisher_->unlockAndPublish();
  }
  if (outcome.wrench_valid && realtime_wrench_publisher_->trylock()) {
    auto & message = realtime_wrench_publisher_->msg_;
    message.header.stamp = time;
    message.wrench.force.x = outcome.wrench_values[0];
    message.wrench.force.y = outcome.wrench_values[1];
    message.wrench.force.z = outcome.wrench_values[2];
    message.wrench.torque.x = outcome.wrench_values[3];
    message.wrench.torque.y = outcome.wrench_values[4];
    message.wrench.torque.z = outcome.wrench_values[5];
    realtime_wrench_publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

CallbackReturn ForceTorqueBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  active_.store(false, std::memory_order_release);
  if (sensor_ != nullptr) {
    sensor_->release_interfaces();
  }
  link_interface_ = nullptr;
  al_state_interface_ = nullptr;
  release_interfaces();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ForceTorqueBroadcaster::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  on_deactivate(state);
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  clear_runtime_resources();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ForceTorqueBroadcaster::on_error(
  const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

void ForceTorqueBroadcaster::publish_calibration()
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  if (calibration_publisher_ == nullptr) {
    return;
  }
  const auto generation = diagnostic_generation_.load(std::memory_order_acquire);
  if (generation == published_diagnostic_generation_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = get_node()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = diagnostic_name_;
  status.hardware_id = sensor_name_;
  const bool valid = calibration_valid_ &&
    !startup_invalidated_.load(std::memory_order_acquire);
  status.level = valid ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = valid ? "Force/torque calibration verified" :
    (startup_invalidated_.load(std::memory_order_acquire) ?
    "Sensor left OP or link was lost; fresh PREOP initialization required" :
    "No valid force/torque calibration snapshot");
  auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(std::move(item));
    };
  add("startup_id", startup_id_);
  add("snapshot_source", snapshot_source_);
  add("snapshot_valid", valid ? "true" : "false");
  add("validity_policy", validity_policy_);
  add("minimum_auxiliary_value", std::to_string(minimum_auxiliary_value_));
  add("maximum_auxiliary_value", std::to_string(maximum_auxiliary_value_));
  for (std::size_t index = 0U; index < decimals_.size(); ++index) {
    add("decimal_" + std::to_string(index + 1U), std::to_string(decimals_[index]));
  }
  for (std::size_t index = 0U; index < unit_codes_.size(); ++index) {
    add("unit_" + std::to_string(index + 1U), std::to_string(unit_codes_[index]));
  }
  message.status.push_back(std::move(status));
  calibration_publisher_->publish(message);
  published_diagnostic_generation_ = generation;
}

void ForceTorqueBroadcaster::clear_runtime_resources()
{
  calibration_timer_.reset();
  calibration_publisher_.reset();
  realtime_raw_publisher_.reset();
  raw_publisher_.reset();
  realtime_wrench_publisher_.reset();
  wrench_publisher_.reset();
  sensor_.reset();
  processor_.reset();
  state_interface_names_.clear();
  link_interface_ = nullptr;
  al_state_interface_ = nullptr;
  calibration_valid_ = false;
  decimals_.clear();
  unit_codes_.clear();
}

}  // namespace rt_force_torque_broadcaster

PLUGINLIB_EXPORT_CLASS(
  rt_force_torque_broadcaster::ForceTorqueBroadcaster,
  controller_interface::ControllerInterface)
