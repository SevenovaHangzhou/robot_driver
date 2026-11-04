#include "periph_bms/node.hpp"

#include "rclcpp/rclcpp.hpp"

#include <exception>
#include <iostream>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = periph_bms::make_node();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & error) {
    std::cerr << "bms_node failed: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
