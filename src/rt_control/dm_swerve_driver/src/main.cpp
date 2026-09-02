#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "dm_swerve_driver/swerve_driver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<dm_swerve_driver::SwerveDriverNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
