#include <array>
#include <memory>
#include <string>
#include <vector>
#include "modbus_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace modbus_tcp_rtu485_led
{
class LedStripNode final : public rclcpp::Node
{
public:
  LedStripNode() : Node("led_strip_node")
  {
    gateway_ip_ = declare_parameter("gateway_ip", std::string("192.168.1.12"));
    ports_ = declare_parameter("gateway_ports", std::vector<int64_t>{502, 503, 504, 505});
    addresses_ = declare_parameter("controller_addresses", std::vector<int64_t>{1, 1, 1, 1});
    response_timeout_ms_ = declare_parameter<int64_t>("response_timeout_ms", 500);
    validate_config(gateway_ip_, ports_, addresses_, response_timeout_ms_);
    for (size_t i = 0; i < 4; ++i) {
      subscriptions_[i] = create_subscription<std_msgs::msg::ColorRGBA>(
        "led" + std::to_string(i) + "/color", rclcpp::QoS(1),
        [this, i](std_msgs::msg::ColorRGBA::ConstSharedPtr message) {send_color(i, *message);});
    }
  }

private:
  void send_color(size_t index, const std_msgs::msg::ColorRGBA & color)
  {
    const std::array<uint8_t, 4> values{
      brightness(color.r), brightness(color.g), brightness(color.b), brightness(color.a)};
    try {
      send_color_tcp(
        gateway_ip_, static_cast<uint16_t>(ports_[index]),
        color_request(++transaction_id_, static_cast<uint8_t>(addresses_[index]), values),
        response_timeout_ms_);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "LED %zu write failed: %s", index, error.what());
    }
  }

  std::string gateway_ip_;
  std::vector<int64_t> ports_, addresses_;
  int64_t response_timeout_ms_{500};
  uint16_t transaction_id_{0};
  std::array<rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr, 4> subscriptions_{};
};
}  // namespace modbus_tcp_rtu485_led

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<modbus_tcp_rtu485_led::LedStripNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("led_strip_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
