#include "periph_imu_lpms/protocol/decoder.hpp"
#include "periph_imu_lpms/ros_conversion.hpp"
#include "infra_socketcan/socket.hpp"
#include "periph_imu_lpms/protocol/socket_can.hpp"
#include "periph_imu_lpms/node.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <linux/can.h>
#include <net/if.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/observation_meta.hpp"
#include "robot_rt_control_interfaces/msg/sensor_status.hpp"
#include "robot_rt_control_interfaces/msg/sensor_status_array.hpp"
#include "robot_system_interfaces/msg/error_info.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace periph_imu_lpms
{
namespace
{

using SteadyClock = std::chrono::steady_clock;

[[nodiscard]] std::chrono::milliseconds positive_duration(
  const std::int64_t value, const char * const parameter_name)
{
  if (value <= 0) {
    throw std::invalid_argument{std::string{parameter_name} + " must be positive"};
  }
  return std::chrono::milliseconds{value};
}

[[nodiscard]] diagnostic_msgs::msg::KeyValue diagnostic_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = std::move(key);
  item.value = std::move(value);
  return item;
}

[[nodiscard]] const char * heartbeat_state_name(const std::uint8_t state) noexcept
{
  switch (state) {
    case 0x00U:
      return "boot-up";
    case 0x04U:
      return "stopped";
    case 0x05U:
      return "operational";
    case 0x7fU:
      return "pre-operational";
    default:
      return "unknown";
  }
}

}  // namespace

class LpmsNav3CanNode final : public rclcpp::Node
{
public:
  explicit LpmsNav3CanNode(const rclcpp::NodeOptions & options)
  : Node("lpms_nav3_can_node", options), node_id_(read_node_id()), decoder_(node_id_)
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    convert_to_ros_ = declare_parameter<bool>("convert_to_ros_convention", false);
    require_heartbeat_ = declare_parameter<bool>("require_heartbeat", true);
    orientation_covariance_ = read_covariance("orientation_covariance");
    angular_velocity_covariance_ = read_covariance("angular_velocity_covariance");
    poll_period_ =
      positive_duration(declare_parameter<std::int64_t>("poll_period_ms", 2), "poll_period_ms");
    reconnect_period_ = positive_duration(
      declare_parameter<std::int64_t>("reconnect_period_ms", 1000), "reconnect_period_ms");
    pdo_timeout_ =
      positive_duration(declare_parameter<std::int64_t>("pdo_timeout_ms", 250), "pdo_timeout_ms");
    assembly_timeout_ = positive_duration(
      declare_parameter<std::int64_t>("assembly_timeout_ms", 50), "assembly_timeout_ms");
    heartbeat_timeout_ = positive_duration(
      declare_parameter<std::int64_t>("heartbeat_timeout_ms", 2500), "heartbeat_timeout_ms");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    const auto diagnostics_topic =
      declare_parameter<std::string>("diagnostics_topic", "~/diagnostics");
    const auto sensor_status_topic =
      declare_parameter<std::string>("sensor_status_topic", "/rt_control/sensors/status");

    if (can_interface_.empty() || can_interface_ == "TBD" || can_interface_.size() >= IFNAMSIZ) {
      throw std::invalid_argument{"can_interface must be explicitly configured"};
    }
    if (
      frame_id_.empty() || frame_id_ == "TBD" || frame_id_.front() == '/' ||
      frame_id_.find_first_of(" \t\r\n") != std::string::npos) {
      throw std::invalid_argument{"frame_id must be explicitly configured without a leading slash"};
    }

    initialise_instance_id();
    imu_publisher_ =
      create_publisher<sensor_msgs::msg::Imu>(imu_topic, robot_interfaces_qos::fast_state());
    magnetic_field_publisher_ = create_publisher<sensor_msgs::msg::MagneticField>(
      "~/mag", robot_interfaces_qos::fast_state());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic, robot_interfaces_qos::diagnostic());
    sensor_status_publisher_ =
      create_publisher<robot_rt_control_interfaces::msg::SensorStatusArray>(
        sensor_status_topic, robot_interfaces_qos::state());

    next_connect_attempt_ = SteadyClock::time_point::min();
    poll_timer_ = create_wall_timer(poll_period_, [this]() { poll_bus(); });
    diagnostics_timer_ =
      create_wall_timer(std::chrono::seconds{1}, [this]() { publish_diagnostics(); });

    RCLCPP_INFO(
      get_logger(), "LPMS-NAV3 CAN driver configured: interface=%s node_id=%u frame_id=%s",
      can_interface_.c_str(), static_cast<unsigned int>(node_id_), frame_id_.c_str());
    try_connect();
  }

private:
  void initialise_instance_id()
  {
    std::random_device random;
    for (auto & byte : producer_instance_id_.uuid) {
      byte = static_cast<std::uint8_t>(random());
    }
  }

  static void fill_error(
    robot_system_interfaces::msg::ErrorInfo & error, std::uint32_t code,
    const std::string & message, bool retryable)
  {
    error.code = code;
    error.message = message;
    error.retryable = retryable;
    error.severity = code == 0U ? robot_system_interfaces::msg::ErrorInfo::OK
                                : robot_system_interfaces::msg::ErrorInfo::FAULT;
    error.source = "rt_control";
    error.detail = "";
  }

  [[nodiscard]] std::uint8_t read_node_id()
  {
    const auto value = declare_parameter<std::int64_t>("node_id", 0);
    if (value < 1 || value > 127) {
      throw std::invalid_argument{"node_id must be in [1, 127]"};
    }
    return static_cast<std::uint8_t>(value);
  }

  void record_socket_error(std::string message)
  {
    if (message != last_socket_error_) {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
      last_socket_error_ = std::move(message);
    }
  }

  void try_connect()
  {
    const auto current_time = SteadyClock::now();
    if (socket_.is_open() || current_time < next_connect_attempt_) {
      return;
    }
    next_connect_attempt_ = current_time + reconnect_period_;
    ++connect_attempt_count_;

    const unsigned int interface_index = if_nametoindex(can_interface_.c_str());
    if (interface_index == 0U) {
      record_socket_error(
        "SocketCAN interface " + can_interface_ + " is unavailable: " + std::strerror(errno));
      return;
    }

    const auto filters = make_can_filters(node_id_);
    infra_socketcan::Socket candidate;
    try {
      candidate.open(can_interface_, {filters.begin(), filters.end()});
    } catch (const std::exception & error) {
      record_socket_error("failed to open " + can_interface_ + ": " + error.what());
      return;
    }

    socket_ = std::move(candidate);
    decoder_.reset();
    has_partial_assembly_ = false;
    has_sample_ = false;
    has_heartbeat_ = false;
    heartbeat_state_ = 0U;
    last_socket_error_.clear();
    RCLCPP_INFO(get_logger(), "connected to SocketCAN interface %s", can_interface_.c_str());
  }

  void disconnect(std::string reason)
  {
    socket_.close();
    decoder_.reset();
    has_partial_assembly_ = false;
    has_sample_ = false;
    has_heartbeat_ = false;
    heartbeat_state_ = 0U;
    record_socket_error(std::move(reason));
    next_connect_attempt_ = SteadyClock::now() + reconnect_period_;
  }

  void poll_bus()
  {
    if (!socket_.is_open()) {
      try_connect();
      return;
    }
    const unsigned int current_interface_index = if_nametoindex(can_interface_.c_str());
    if (
      current_interface_index == 0U ||
      static_cast<int>(current_interface_index) != socket_.interface_index()) {
      disconnect("SocketCAN interface disappeared or was replaced: " + can_interface_);
      return;
    }

    for (std::size_t iteration = 0U; iteration < 256U; ++iteration) {
      can_frame raw_frame{};
      const int received = socket_.receive(raw_frame);
      if (received == 0) {
        return;
      }
      if (received == -EIO) {
        ++malformed_frame_count_;
        continue;
      }
      if (received < 0) {
        disconnect("SocketCAN receive failed: " + std::string{std::strerror(-received)});
        return;
      }

      CanFrame frame;
      frame.is_extended = (raw_frame.can_id & CAN_EFF_FLAG) != 0U;
      frame.is_remote = (raw_frame.can_id & CAN_RTR_FLAG) != 0U;
      frame.is_error = (raw_frame.can_id & CAN_ERR_FLAG) != 0U;
      frame.id = raw_frame.can_id & (frame.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
      frame.dlc = raw_frame.can_dlc;
      std::copy(std::begin(raw_frame.data), std::end(raw_frame.data), frame.data.begin());

      const auto receive_time = SteadyClock::now();
      const auto node = static_cast<std::uint32_t>(node_id_);
      const bool is_pdo = frame.id == 0x180U + node || frame.id == 0x280U + node ||
                          frame.id == 0x380U + node || frame.id == 0x480U + node;
      if (
        is_pdo && has_partial_assembly_ &&
        receive_time - assembly_started_time_ > assembly_timeout_) {
        decoder_.reset();
        has_partial_assembly_ = false;
        ++dropped_assembly_count_;
      }

      const bool starts_new_assembly = is_pdo && !has_partial_assembly_;
      const auto result = decoder_.consume(frame);
      if (!result.recognized) {
        continue;
      }
      ++recognized_frame_count_;
      if (result.malformed) {
        ++malformed_frame_count_;
        continue;
      }
      if (is_pdo && starts_new_assembly) {
        assembly_started_time_ = receive_time;
        has_partial_assembly_ = true;
      }
      if (result.heartbeat_state.has_value()) {
        heartbeat_state_ = *result.heartbeat_state;
        last_heartbeat_time_ = receive_time;
        has_heartbeat_ = true;
      }
      if (result.sample.has_value()) {
        has_partial_assembly_ = false;
        last_sample_time_ = receive_time;
        has_sample_ = true;
        ++published_sample_count_;
        publish_sample(*result.sample);
      }
    }
  }

  void publish_sample(const ImuSample & raw_sample)
  {
    const auto sample = convert_to_ros_convention(raw_sample, convert_to_ros_);
    auto imu_message =
      make_imu_message(sample, orientation_covariance_, angular_velocity_covariance_);
    auto magnetic_field_message = make_magnetic_field_message(sample);
    const auto stamp = now();
    last_sample_stamp_ = stamp;
    imu_message.header.stamp = stamp;
    imu_message.header.frame_id = frame_id_;
    magnetic_field_message.header = imu_message.header;
    imu_publisher_->publish(imu_message);
    magnetic_field_publisher_->publish(magnetic_field_message);
  }

  [[nodiscard]] static double age_ms(
    const SteadyClock::time_point timestamp, const SteadyClock::time_point now) noexcept
  {
    return std::chrono::duration<double, std::milli>{now - timestamp}.count();
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name() + std::string{": LPMS-NAV3 CAN"};
    status.hardware_id = "lpms-nav3-node-" + std::to_string(node_id_) + "@" + can_interface_;

    const auto current_time = SteadyClock::now();
    const double sample_age = has_sample_ ? age_ms(last_sample_time_, current_time) : -1.0;
    const double heartbeat_age = has_heartbeat_ ? age_ms(last_heartbeat_time_, current_time) : -1.0;
    if (!socket_.is_open()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "SocketCAN interface unavailable";
    } else if (!has_sample_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for complete TPDO set";
    } else if (sample_age > static_cast<double>(pdo_timeout_.count())) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "TPDO data stale";
    } else if (
      require_heartbeat_ &&
      (!has_heartbeat_ || heartbeat_age > static_cast<double>(heartbeat_timeout_.count()))) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "CANopen heartbeat missing or stale";
    } else if (require_heartbeat_ && heartbeat_state_ != 0x05U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "CANopen node is not operational";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "LPMS-NAV3 CAN data healthy";
    }

    status.values.push_back(diagnostic_value("can_interface", can_interface_));
    status.values.push_back(diagnostic_value("node_id", std::to_string(node_id_)));
    status.values.push_back(
      diagnostic_value("socket_connected", socket_.is_open() ? "true" : "false"));
    status.values.push_back(diagnostic_value("sample_age_ms", std::to_string(sample_age)));
    status.values.push_back(diagnostic_value("heartbeat_age_ms", std::to_string(heartbeat_age)));
    status.values.push_back(
      diagnostic_value("heartbeat_state", heartbeat_state_name(heartbeat_state_)));
    status.values.push_back(
      diagnostic_value("published_samples", std::to_string(published_sample_count_)));
    status.values.push_back(
      diagnostic_value("recognized_frames", std::to_string(recognized_frame_count_)));
    status.values.push_back(
      diagnostic_value("malformed_frames", std::to_string(malformed_frame_count_)));
    status.values.push_back(
      diagnostic_value("dropped_assemblies", std::to_string(dropped_assembly_count_)));
    status.values.push_back(
      diagnostic_value("connect_attempts", std::to_string(connect_attempt_count_)));
    status.values.push_back(diagnostic_value("last_socket_error", last_socket_error_));

    robot_rt_control_interfaces::msg::SensorStatusArray public_status;
    public_status.header.stamp = array.header.stamp;
    robot_rt_control_interfaces::msg::SensorStatus imu_status;
    imu_status.sensor_id = "lpms_nav3";
    imu_status.module_id = "lpms_nav3_can";
    imu_status.data_topic = "/imu/data";
    imu_status.observation.stamp = last_sample_stamp_;
    imu_status.observation.time_source =
      robot_rt_control_interfaces::msg::ObservationMeta::TIME_SOURCE_PUBLICATION;
    imu_status.observation.valid = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK;
    imu_status.observation.source_instance_id = producer_instance_id_;
    imu_status.observation.sample_sequence = published_sample_count_;
    if (imu_status.observation.valid) {
      imu_status.measurement_state =
        robot_rt_control_interfaces::msg::SensorStatus::MEASUREMENT_VALID;
      fill_error(imu_status.observation.error, 0U, "", false);
    } else {
      imu_status.measurement_state =
        socket_.is_open()
          ? robot_rt_control_interfaces::msg::SensorStatus::MEASUREMENT_STALE
          : robot_rt_control_interfaces::msg::SensorStatus::MEASUREMENT_COMMUNICATION_ERROR;
      const std::uint32_t code = socket_.is_open() ? 1140U : 1100U;
      fill_error(imu_status.observation.error, code, status.message, true);
      imu_status.fault_detected_at = array.header.stamp;
      imu_status.fault_detected_time_valid = true;
    }
    imu_status.error = imu_status.observation.error;
    public_status.sensors.push_back(std::move(imu_status));
    sensor_status_publisher_->publish(public_status);

    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  [[nodiscard]] std::array<double, 9U> read_covariance(const std::string & name)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Row-major 3x3 covariance; all zeros means unknown";
    const auto values =
      declare_parameter<std::vector<double>>(name, std::vector<double>(9U, 0.0), descriptor);
    if (values.size() != 9U || !std::all_of(values.begin(), values.end(), [](double value) {
          return std::isfinite(value);
        })) {
      throw std::invalid_argument{name + " must contain exactly 9 finite numbers"};
    }
    std::array<double, 9U> result{};
    std::copy(values.begin(), values.end(), result.begin());
    return result;
  }

  std::array<double, 9U> orientation_covariance_{};
  std::array<double, 9U> angular_velocity_covariance_{};
  std::string can_interface_;
  std::string frame_id_;
  const std::uint8_t node_id_;
  Decoder decoder_;
  bool convert_to_ros_{false};
  bool require_heartbeat_{true};
  std::chrono::milliseconds poll_period_{2};
  std::chrono::milliseconds reconnect_period_{1000};
  std::chrono::milliseconds pdo_timeout_{250};
  std::chrono::milliseconds assembly_timeout_{50};
  std::chrono::milliseconds heartbeat_timeout_{2500};
  infra_socketcan::Socket socket_;
  SteadyClock::time_point next_connect_attempt_{};
  SteadyClock::time_point last_sample_time_{};
  SteadyClock::time_point last_heartbeat_time_{};
  SteadyClock::time_point assembly_started_time_{};
  bool has_sample_{false};
  bool has_heartbeat_{false};
  bool has_partial_assembly_{false};
  std::uint8_t heartbeat_state_{0U};
  std::uint64_t published_sample_count_{0U};
  std::uint64_t recognized_frame_count_{0U};
  std::uint64_t malformed_frame_count_{0U};
  std::uint64_t dropped_assembly_count_{0U};
  std::uint64_t connect_attempt_count_{0U};
  std::string last_socket_error_;
  builtin_interfaces::msg::Time last_sample_stamp_;
  unique_identifier_msgs::msg::UUID producer_instance_id_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<robot_rt_control_interfaces::msg::SensorStatusArray>::SharedPtr
    sensor_status_publisher_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

rclcpp::Node::SharedPtr make_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<LpmsNav3CanNode>(options);
}

}  // namespace periph_imu_lpms
