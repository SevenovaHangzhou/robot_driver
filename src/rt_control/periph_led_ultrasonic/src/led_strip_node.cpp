#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "infra_modbus/client.hpp"
#include "periph_led_ultrasonic/protocol/modbus_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_rt_control_interfaces/srv/set_led_rgbw.hpp"
#include "robot_system_interfaces/msg/error_info.hpp"

namespace periph_led_ultrasonic
{
class LedStripNode final : public rclcpp::Node
{
public:
  LedStripNode() : Node("led_strip_node")
  {
    gateway_ip_ = declare_parameter("gateway_ip", std::string("192.168.1.12"));
    ports_ = declare_parameter("gateway_ports", std::vector<int64_t>{502, 502, 502, 502, 502, 502});
    addresses_ = declare_parameter("controller_addresses", std::vector<int64_t>{1, 2, 3, 4, 5, 6});
    response_timeout_ms_ = declare_parameter<int64_t>("response_timeout_ms", 500);
    exit_response_timeout_ms_ = declare_parameter<int64_t>("exit_response_timeout_ms", 100);
    service_name_ = declare_parameter("service_name", std::string("/led/set_rgbw"));
    exit_color_enabled_ = declare_parameter("exit_color_enabled", true);
    validate_led_config(ports_, addresses_, response_timeout_ms_);
    if (exit_response_timeout_ms_ < 1 || exit_response_timeout_ms_ > 60000) {
      throw std::invalid_argument("exit_response_timeout_ms must be 1..60000");
    }
    // One long-lived connection per gateway port; controllers on the same port share it.
    for (const auto port : ports_) {
      const auto key = static_cast<uint16_t>(port);
      if (clients_.count(key) == 0U) {
        clients_.emplace(
          key, std::make_unique<infra_modbus::Client>(
                 gateway_ip_, key, std::chrono::milliseconds(response_timeout_ms_)));
      }
    }
    if (service_name_.empty()) {
      throw std::invalid_argument("service_name must not be empty");
    }
    service_ = create_service<robot_rt_control_interfaces::srv::SetLedRgbw>(
      service_name_, [this](
                       robot_rt_control_interfaces::srv::SetLedRgbw::Request::SharedPtr request,
                       robot_rt_control_interfaces::srv::SetLedRgbw::Response::SharedPtr response) {
        set_color(*request, *response);
      });
  }

  void set_normal_exit_color() noexcept { set_all_exit_colors(led_exit_color(false), "off"); }

  void set_abnormal_exit_color() noexcept { set_all_exit_colors(led_exit_color(true), "red"); }

private:
  static bool valid_brightness(float value)
  {
    return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
  }

  static void fill_error(
    robot_system_interfaces::msg::ErrorInfo & error, uint32_t code, const std::string & message,
    bool retryable)
  {
    error.code = code;
    error.message = message;
    error.retryable = retryable;
    error.severity = code == 0U ? robot_system_interfaces::msg::ErrorInfo::OK
                                : robot_system_interfaces::msg::ErrorInfo::FAULT;
    error.source = "rt_control";
    error.detail = "";
  }

  void set_color(
    const robot_rt_control_interfaces::srv::SetLedRgbw::Request & request,
    robot_rt_control_interfaces::srv::SetLedRgbw::Response & response)
  {
    const std::array<float, 4> requested{request.red, request.green, request.blue, request.white};
    if (
      request.strip_id >= kLedControllerCount ||
      !std::all_of(requested.begin(), requested.end(), valid_brightness)) {
      response.accepted = false;
      response.write_acknowledged = false;
      response.completed_at = now();
      fill_error(response.error, 4U, "invalid strip_id or RGBW value", false);
      return;
    }

    response.accepted = true;
    const std::array<uint8_t, 4> values{
      brightness(request.red), brightness(request.green), brightness(request.blue),
      brightness(request.white)};
    try {
      const auto index = static_cast<size_t>(request.strip_id);
      check_color_ack(clients_.at(static_cast<uint16_t>(ports_[index]))
                        ->transact(static_cast<uint8_t>(addresses_[index]), color_pdu(values)));
      response.write_acknowledged = true;
      fill_error(response.error, 0U, "LED write acknowledged", false);
    } catch (const std::exception & error) {
      response.write_acknowledged = false;
      fill_error(response.error, 1100U, error.what(), true);
      RCLCPP_WARN(get_logger(), "LED %u write failed: %s", request.strip_id, error.what());
    }
    response.completed_at = now();
  }

  void set_all_exit_colors(const std::array<uint8_t, 4> & values, const char * color_name) noexcept
  {
    if (!exit_color_enabled_) {
      return;
    }
    for (size_t index = 0; index < kLedControllerCount; ++index) {
      try {
        check_color_ack(clients_.at(static_cast<uint16_t>(ports_[index]))
                          ->transact(
                            static_cast<uint8_t>(addresses_[index]), color_pdu(values),
                            std::chrono::milliseconds(exit_response_timeout_ms_)));
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Failed to set LED %zu to %s during exit: %s", index, color_name,
          error.what());
      }
    }
  }

  std::string gateway_ip_;
  std::string service_name_;
  std::vector<int64_t> ports_;
  std::vector<int64_t> addresses_;
  int64_t response_timeout_ms_{500};
  int64_t exit_response_timeout_ms_{100};
  bool exit_color_enabled_{true};
  std::map<uint16_t, std::unique_ptr<infra_modbus::Client>> clients_;
  rclcpp::Service<robot_rt_control_interfaces::srv::SetLedRgbw>::SharedPtr service_;
};
}  // namespace periph_led_ultrasonic

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<periph_led_ultrasonic::LedStripNode> node;
  try {
    node = std::make_shared<periph_led_ultrasonic::LedStripNode>();
    rclcpp::spin(node);
    node->set_normal_exit_color();
  } catch (const std::exception & error) {
    if (node) {
      node->set_abnormal_exit_color();
    }
    RCLCPP_ERROR(rclcpp::get_logger("led_strip_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    if (node) {
      node->set_abnormal_exit_color();
    }
    RCLCPP_ERROR(rclcpp::get_logger("led_strip_node"), "Unknown fatal error");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
