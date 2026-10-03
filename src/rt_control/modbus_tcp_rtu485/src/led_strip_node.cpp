#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "modbus_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_rt_control_interfaces/srv/set_led_rgbw.hpp"
#include "robot_system_interfaces/msg/error_info.hpp"

namespace modbus_tcp_rtu485
{
class LedStripNode final : public rclcpp::Node
{
public:
  LedStripNode() : Node("led_strip_node")
  {
    gateway_ip_ = declare_parameter("gateway_ip", std::string("192.168.1.12"));
    ports_ = declare_parameter(
      "gateway_ports", std::vector<int64_t>{502, 502, 502, 502, 502, 502});
    addresses_ = declare_parameter(
      "controller_addresses", std::vector<int64_t>{1, 2, 3, 4, 5, 6});
    response_timeout_ms_ = declare_parameter<int64_t>("response_timeout_ms", 500);
    service_name_ = declare_parameter("service_name", std::string("/led/set_rgbw"));
    validate_led_config(gateway_ip_, ports_, addresses_, response_timeout_ms_);
    if (service_name_.empty()) {
      throw std::invalid_argument("service_name must not be empty");
    }
    service_ = create_service<robot_rt_control_interfaces::srv::SetLedRgbw>(
      service_name_,
      [this](
        robot_rt_control_interfaces::srv::SetLedRgbw::Request::SharedPtr request,
        robot_rt_control_interfaces::srv::SetLedRgbw::Response::SharedPtr response)
      {
        set_color(*request, *response);
      });
  }

private:
  static bool valid_brightness(float value)
  {
    return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
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

  void set_color(
    const robot_rt_control_interfaces::srv::SetLedRgbw::Request & request,
    robot_rt_control_interfaces::srv::SetLedRgbw::Response & response)
  {
    const std::array<float, 4> requested{
      request.red, request.green, request.blue, request.white};
    if (request.strip_id >= kLedControllerCount ||
      !std::all_of(requested.begin(), requested.end(), valid_brightness))
    {
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
      send_color_tcp(
        gateway_ip_, static_cast<uint16_t>(ports_[index]),
        color_request(++transaction_id_, static_cast<uint8_t>(addresses_[index]), values),
        response_timeout_ms_);
      response.write_acknowledged = true;
      fill_error(response.error, 0U, "LED write acknowledged", false);
    } catch (const std::exception & error) {
      response.write_acknowledged = false;
      fill_error(response.error, 1100U, error.what(), true);
      RCLCPP_WARN(get_logger(), "LED %u write failed: %s", request.strip_id, error.what());
    }
    response.completed_at = now();
  }

  std::string gateway_ip_;
  std::string service_name_;
  std::vector<int64_t> ports_;
  std::vector<int64_t> addresses_;
  int64_t response_timeout_ms_{500};
  uint16_t transaction_id_{0};
  rclcpp::Service<robot_rt_control_interfaces::srv::SetLedRgbw>::SharedPtr service_;
};
}  // namespace modbus_tcp_rtu485

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<modbus_tcp_rtu485::LedStripNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("led_strip_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
