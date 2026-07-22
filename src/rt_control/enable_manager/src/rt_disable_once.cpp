#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/rt_enable.hpp"

namespace
{
constexpr auto kTotalDeadline = std::chrono::seconds(30);
constexpr auto kServiceProbe = std::chrono::milliseconds(100);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared("rt_disable_once");
  const auto client = node->create_client<robot_interfaces::srv::RtEnable>("/rt/disable");
  const auto deadline = std::chrono::steady_clock::now() + kTotalDeadline;

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (client->wait_for_service(kServiceProbe)) {
      break;
    }
  }
  if (!rclcpp::ok() || !client->service_is_ready()) {
    std::cerr << "UNCLEAN_SHUTDOWN: /rt/disable unavailable within 30 seconds\n";
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const auto request = std::make_shared<robot_interfaces::srv::RtEnable::Request>();
  auto future = client->async_send_request(request);
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) {
    std::cerr << "UNCLEAN_SHUTDOWN: /rt/disable deadline expired before response wait\n";
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  const auto result = rclcpp::spin_until_future_complete(node, future, deadline - now);
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    std::cerr << "UNCLEAN_SHUTDOWN: /rt/disable did not return within 30 seconds\n";
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const auto response = future.get();
  std::cerr << "rt_control shutdown disable result: ok=" << std::boolalpha << response->ok
            << " stage=" << response->stage
            << " failed_batch=" << static_cast<int>(response->failed_batch)
            << " failed_joint=" << response->failed_joint
            << " status_word=" << response->status_word << '\n';
  const bool clean = response->ok;
  rclcpp::shutdown();
  return clean ? EXIT_SUCCESS : EXIT_FAILURE;
}
