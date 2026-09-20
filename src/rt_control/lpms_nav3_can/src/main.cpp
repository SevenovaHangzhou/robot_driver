#include <cstdio>
#include <exception>

#include "lpms_nav3_can/node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(lpms_nav3_can::make_node(rclcpp::NodeOptions{}));
  } catch (const std::exception & error) {
    const bool shutdown_requested = !rclcpp::ok();
    if (!shutdown_requested) {
      std::fprintf(stderr, "lpms_nav3_can_node: %s\n", error.what());
    }
    rclcpp::shutdown();
    return shutdown_requested ? 0 : 1;
  }
  rclcpp::shutdown();
  return 0;
}
