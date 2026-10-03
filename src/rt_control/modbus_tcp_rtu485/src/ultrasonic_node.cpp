#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "modbus_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/observation_meta.hpp"
#include "robot_rt_control_interfaces/msg/sensor_status.hpp"
#include "robot_rt_control_interfaces/msg/sensor_status_array.hpp"
#include "robot_system_interfaces/msg/error_info.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "ultrasonic_protocol.hpp"
#include "ultrasonic_range.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace modbus_tcp_rtu485
{
class UltrasonicNode final : public rclcpp::Node
{
public:
  UltrasonicNode() : Node("ultrasonic_node")
  {
    gateway_ip_ = declare_parameter("gateway_ip", std::string("192.168.1.12"));
    gateway_port_ = declare_parameter<int64_t>("gateway_port", 504);
    unit_ids_ = declare_parameter<std::vector<int64_t>>("unit_ids", {1, 6});
    response_timeout_ms_ = declare_parameter<int64_t>("response_timeout_ms", 500);
    poll_interval_ms_ = declare_parameter<int64_t>("poll_interval_ms", 300);
    poll_enabled_ = declare_parameter("poll_enabled", true);
    min_range_m_ = declare_parameter("min_range_m", 0.01);
    max_range_m_ = declare_parameter("max_range_m", 3.5);
    field_of_view_rad_ = declare_parameter("field_of_view_rad", 0.6981317008);
    frame_ids_ = declare_parameter<std::vector<std::string>>(
      "frame_ids",
      {"ultrasonic_channel_1_link", "ultrasonic_channel_2_link",
        "ultrasonic_channel_3_link", "ultrasonic_channel_4_link",
        "ultrasonic_channel_5_link", "ultrasonic_channel_6_link",
        "ultrasonic_channel_7_link", "ultrasonic_channel_8_link"});
    validate_parameters();
    initialise_instance_id();

    for (size_t i = 0; i < range_publishers_.size(); ++i) {
      range_publishers_[i] = create_publisher<sensor_msgs::msg::Range>(
        "/ultrasonic/channel" + std::to_string(i + 1) + "/range",
        robot_interfaces_qos::fast_state());
    }
    for (size_t i = 0; i < raw_publishers_.size(); ++i) {
      raw_publishers_[i] = create_publisher<std_msgs::msg::UInt16MultiArray>(
        "/ultrasonic/unit" + std::to_string(unit_ids_[i]) + "/raw", 10);
    }
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/ultrasonic/diagnostics", robot_interfaces_qos::diagnostic());
    status_publisher_ =
      create_publisher<robot_rt_control_interfaces::msg::SensorStatusArray>(
      "/rt_control/sensors/status", robot_interfaces_qos::state());

    if (poll_enabled_) {
      timer_ = create_wall_timer(
        std::chrono::milliseconds(poll_interval_ms_), [this]() {poll();});
    }
  }

private:
  void validate_parameters() const
  {
    validate_e08_unit_ids(unit_ids_);
    for (const auto unit_id : unit_ids_) {
      validate_endpoint(gateway_ip_, gateway_port_, unit_id, response_timeout_ms_);
    }
    if (poll_interval_ms_ < 20 || poll_interval_ms_ > 60000) {
      throw std::invalid_argument("poll_interval_ms must be 20..60000");
    }
    validate_ultrasonic_config(
      frame_ids_, static_cast<float>(field_of_view_rad_),
      static_cast<float>(min_range_m_), static_cast<float>(max_range_m_));
  }

  void initialise_instance_id()
  {
    std::random_device random;
    for (auto & byte : producer_instance_id_.uuid) {
      byte = static_cast<uint8_t>(random());
    }
  }

  static diagnostic_msgs::msg::KeyValue key_value(std::string key, std::string value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = std::move(key);
    item.value = std::move(value);
    return item;
  }

  std::string sensor_id(size_t channel_index) const
  {
    const size_t module_index = channel_index / 4U;
    const size_t local_channel = channel_index % 4U;
    return "DYP-E084F-V2.0/unit" + std::to_string(unit_ids_[module_index]) +
      "/channel" + std::to_string(local_channel + 1U);
  }

  diagnostic_msgs::msg::DiagnosticStatus channel_diagnostic(
    size_t index, uint16_t raw, const UltrasonicReading & reading) const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = reading.diagnostic_level;
    status.name = "ultrasonic/channel" + std::to_string(index + 1U);
    status.hardware_id = sensor_id(index);
    status.message = reading.status;
    status.values.push_back(key_value("raw", std::to_string(raw)));
    if (std::isfinite(reading.range_m)) {
      status.values.push_back(key_value("distance_mm", std::to_string(raw)));
    }
    return status;
  }

  static void fill_error(
    robot_system_interfaces::msg::ErrorInfo & error, uint32_t code,
    const std::string & message, bool retryable)
  {
    error.code = code;
    error.message = message;
    error.retryable = retryable;
    error.severity = code == 0U ?
      robot_system_interfaces::msg::ErrorInfo::OK :
      robot_system_interfaces::msg::ErrorInfo::FAULT;
    error.source = "rt_control";
    error.detail = "";
  }

  robot_rt_control_interfaces::msg::SensorStatus make_status(
    size_t index, const rclcpp::Time & sample_time, uint8_t measurement_state,
    bool valid, const std::string & message, uint32_t error_code)
  {
    using ObservationMeta = robot_rt_control_interfaces::msg::ObservationMeta;
    robot_rt_control_interfaces::msg::SensorStatus status;
    status.sensor_id = sensor_id(index);
    status.module_id = "ultrasonic_unit" + std::to_string(unit_ids_[index / 4U]);
    status.data_topic = "/ultrasonic/channel" + std::to_string(index + 1U) + "/range";
    status.observation.stamp = sample_time;
    status.observation.time_source = ObservationMeta::TIME_SOURCE_READ_COMPLETE;
    status.observation.valid = valid;
    status.observation.source_instance_id = producer_instance_id_;
    status.observation.sample_sequence = sample_time.nanoseconds() > 0 ?
      ++sample_sequences_[index] : sample_sequences_[index];
    fill_error(status.observation.error, error_code, message, error_code != 0U);
    status.measurement_state = measurement_state;
    if (error_code != 0U) {
      status.fault_detected_at = now();
      status.fault_detected_time_valid = true;
    }
    status.error = status.observation.error;
    return status;
  }

  void poll_module(
    size_t module_index,
    diagnostic_msgs::msg::DiagnosticArray & diagnostics,
    robot_rt_control_interfaces::msg::SensorStatusArray & statuses)
  {
    const size_t channel_offset = module_index * 4U;
    try {
      const auto values = read_holding_registers_tcp(
        gateway_ip_, static_cast<uint16_t>(gateway_port_), ++transaction_id_,
        static_cast<uint8_t>(unit_ids_[module_index]), 0x0106, 4, response_timeout_ms_);
      const auto stamp = now();
      std_msgs::msg::UInt16MultiArray raw_message;
      raw_message.data.assign(values.begin(), values.end());
      raw_publishers_[module_index]->publish(raw_message);

      for (size_t local = 0; local < values.size(); ++local) {
        const size_t index = channel_offset + local;
        const auto reading = decode_ultrasonic(
          values[local], static_cast<float>(min_range_m_), static_cast<float>(max_range_m_));
        range_publishers_[index]->publish(make_range_message(
            stamp, frame_ids_[index], reading, static_cast<float>(field_of_view_rad_),
            static_cast<float>(min_range_m_), static_cast<float>(max_range_m_)));
        diagnostics.status.push_back(channel_diagnostic(index, values[local], reading));
        uint8_t state = robot_rt_control_interfaces::msg::SensorStatus::MEASUREMENT_VALID;
        bool valid = std::isfinite(reading.range_m);
        uint32_t code = 0U;
        if (reading.status == "no_target") {
          state = robot_rt_control_interfaces::msg::SensorStatus::MEASUREMENT_NO_TARGET;
          valid = true;
        } else if (reading.diagnostic_level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
          state = robot_rt_control_interfaces::msg::SensorStatus::MEASUREMENT_INVALID;
          valid = false;
          code = 1140U;
        }
        statuses.sensors.push_back(make_status(
            index, stamp, state, valid, reading.status, code));
      }
    } catch (const std::exception & error) {
      diagnostic_msgs::msg::DiagnosticStatus diagnostic;
      diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostic.name = "ultrasonic/unit" + std::to_string(unit_ids_[module_index]);
      diagnostic.hardware_id = gateway_ip_ + ":" + std::to_string(gateway_port_);
      diagnostic.message = "communication_error";
      diagnostic.values.push_back(key_value("detail", error.what()));
      diagnostics.status.push_back(std::move(diagnostic));
      for (size_t local = 0; local < 4U; ++local) {
        statuses.sensors.push_back(make_status(
            channel_offset + local,
            rclcpp::Time(0, 0, get_clock()->get_clock_type()),
            robot_rt_control_interfaces::msg::SensorStatus::MEASUREMENT_COMMUNICATION_ERROR,
            false, error.what(), 1100U));
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ultrasonic unit %ld read failed: %s",
        static_cast<long>(unit_ids_[module_index]), error.what());
    }
  }

  void poll()
  {
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    robot_rt_control_interfaces::msg::SensorStatusArray statuses;
    for (size_t module_index = 0; module_index < unit_ids_.size(); ++module_index) {
      poll_module(module_index, diagnostics, statuses);
    }
    diagnostics.header.stamp = now();
    statuses.header.stamp = diagnostics.header.stamp;
    diagnostic_publisher_->publish(diagnostics);
    status_publisher_->publish(statuses);
  }

  std::string gateway_ip_;
  int64_t gateway_port_{504};
  std::vector<int64_t> unit_ids_{1, 6};
  int64_t response_timeout_ms_{500};
  int64_t poll_interval_ms_{300};
  bool poll_enabled_{true};
  double min_range_m_{0.01};
  double max_range_m_{3.5};
  double field_of_view_rad_{0.6981317008};
  std::vector<std::string> frame_ids_;
  uint16_t transaction_id_{0};
  unique_identifier_msgs::msg::UUID producer_instance_id_;
  std::array<uint64_t, kUltrasonicChannelCount> sample_sequences_{};
  std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, kUltrasonicChannelCount>
    range_publishers_{};
  std::array<rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr, 2>
    raw_publishers_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Publisher<robot_rt_control_interfaces::msg::SensorStatusArray>::SharedPtr
    status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace modbus_tcp_rtu485

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<modbus_tcp_rtu485::UltrasonicNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("ultrasonic_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
