#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rt_control_interfaces/srv/rt_enable.hpp"

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  using Service = rt_control_interfaces::srv::RtEnable;
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared("damiao_head_disable_once");
  const auto client = node->create_client<Service>("/rt/head/disable");
  if (!client->wait_for_service(10s)) {
    std::cerr << "UNCLEAN_SHUTDOWN: /rt/head/disable unavailable for DaMiao head\n";
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  auto future = client->async_send_request(std::make_shared<Service::Request>());
  const auto result = rclcpp::spin_until_future_complete(node, future, 10s);
  if (result != rclcpp::FutureReturnCode::SUCCESS || !future.get()->ok) {
    std::cerr << "UNCLEAN_SHUTDOWN: DaMiao head did not confirm disabled\n";
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
