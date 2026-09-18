#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "modbus_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "ultrasonic_protocol.hpp"
#include "ultrasonic_range.hpp"

namespace modbus_tcp_rtu485
{
class UltrasonicNode final : public rclcpp::Node
{
public:
  UltrasonicNode() : Node("ultrasonic_node")
  {
    gateway_ip_ = declare_parameter("gateway_ip", std::string("192.168.1.12"));
    gateway_port_ = declare_parameter<int64_t>("gateway_port", 504);
    unit_id_ = declare_parameter<int64_t>("unit_id", 1);
    response_timeout_ms_ = declare_parameter<int64_t>("response_timeout_ms", 500);
    poll_interval_ms_ = declare_parameter<int64_t>("poll_interval_ms", 300);
    poll_enabled_ = declare_parameter("poll_enabled", true);
    min_range_m_ = declare_parameter("min_range_m", 0.01);
    max_range_m_ = declare_parameter("max_range_m", 3.5);
    field_of_view_rad_ = declare_parameter("field_of_view_rad", 1.0471975512);
    frame_ids_ = declare_parameter<std::vector<std::string>>(
      "frame_ids",
      {"ultrasonic_channel_1_link", "ultrasonic_channel_2_link",
        "ultrasonic_channel_3_link", "ultrasonic_channel_4_link"});
    validate_parameters();

    for (size_t i = 0; i < range_publishers_.size(); ++i) {
      range_publishers_[i] = create_publisher<sensor_msgs::msg::Range>(
        "ultrasonic/channel" + std::to_string(i + 1) + "/range", rclcpp::SensorDataQoS());
    }
    raw_publisher_ = create_publisher<std_msgs::msg::UInt16MultiArray>("ultrasonic/raw", 10);
    diagnostic_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("ultrasonic/diagnostics", 10);

    if (poll_enabled_) {
      timer_ = create_wall_timer(
        std::chrono::milliseconds(poll_interval_ms_), [this]() {poll();});
    }
  }

private:
  void validate_parameters() const
  {
    validate_endpoint(gateway_ip_, gateway_port_, unit_id_, response_timeout_ms_);
    if (poll_interval_ms_ < 20 || poll_interval_ms_ > 60000) {
      throw std::invalid_argument("poll_interval_ms must be 20..60000");
    }
    validate_ultrasonic_config(
      frame_ids_, static_cast<float>(field_of_view_rad_),
      static_cast<float>(min_range_m_), static_cast<float>(max_range_m_));
  }

  static diagnostic_msgs::msg::KeyValue key_value(std::string key, std::string value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = std::move(key);
    item.value = std::move(value);
    return item;
  }

  diagnostic_msgs::msg::DiagnosticStatus channel_diagnostic(
    size_t index, uint16_t raw, const UltrasonicReading & reading) const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = reading.diagnostic_level;
    status.name = "ultrasonic/channel" + std::to_string(index + 1);
    status.hardware_id = "DYP-E084F-V2.0/channel" + std::to_string(index + 1);
    status.message = reading.status;
    status.values.push_back(key_value("raw", std::to_string(raw)));
    if (std::isfinite(reading.range_m)) {
      status.values.push_back(key_value("distance_mm", std::to_string(raw)));
    }
    return status;
  }

  void publish_failure(const rclcpp::Time & stamp, const std::string & message)
  {
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.name = "ultrasonic/gateway";
    status.hardware_id = gateway_ip_ + ":" + std::to_string(gateway_port_);
    status.message = "communication_error";
    status.values.push_back(key_value("detail", message));
    diagnostics.status.push_back(std::move(status));
    diagnostic_publisher_->publish(diagnostics);
  }

  void poll()
  {
    try {
      const auto values = read_holding_registers_tcp(
        gateway_ip_, static_cast<uint16_t>(gateway_port_), ++transaction_id_,
        static_cast<uint8_t>(unit_id_), 0x0106, 4, response_timeout_ms_);
      const auto stamp = now();

      std_msgs::msg::UInt16MultiArray raw_message;
      raw_message.data.assign(values.begin(), values.end());
      raw_publisher_->publish(raw_message);

      diagnostic_msgs::msg::DiagnosticArray diagnostics;
      diagnostics.header.stamp = stamp;
      for (size_t i = 0; i < values.size(); ++i) {
        const auto reading = decode_ultrasonic(
          values[i], static_cast<float>(min_range_m_), static_cast<float>(max_range_m_));
        const auto range = make_range_message(
          stamp, frame_ids_[i], reading, static_cast<float>(field_of_view_rad_),
          static_cast<float>(min_range_m_), static_cast<float>(max_range_m_));
        range_publishers_[i]->publish(range);
        diagnostics.status.push_back(channel_diagnostic(i, values[i], reading));
      }
      diagnostic_publisher_->publish(diagnostics);
    } catch (const std::exception & error) {
      publish_failure(now(), error.what());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ultrasonic read failed: %s", error.what());
    }
  }

  std::string gateway_ip_;
  int64_t gateway_port_{504};
  int64_t unit_id_{1};
  int64_t response_timeout_ms_{500};
  int64_t poll_interval_ms_{300};
  bool poll_enabled_{true};
  double min_range_m_{0.01};
  double max_range_m_{3.5};
  double field_of_view_rad_{1.0471975512};
  std::vector<std::string> frame_ids_;
  uint16_t transaction_id_{0};
  std::array<
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr,
    4> range_publishers_{};
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr raw_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
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
