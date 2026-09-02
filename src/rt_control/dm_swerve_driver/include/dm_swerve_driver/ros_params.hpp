#ifndef DM_SWERVE_DRIVER__ROS_PARAMS_HPP_
#define DM_SWERVE_DRIVER__ROS_PARAMS_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "dm_swerve_driver/params.hpp"

namespace dm_swerve_driver {

void declare_driver_parameters(rclcpp_lifecycle::LifecycleNode & node);
[[nodiscard]] DriverParameters load_driver_parameters(
  const rclcpp_lifecycle::LifecycleNode & node);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__ROS_PARAMS_HPP_
