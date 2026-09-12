#ifndef DM_SWERVE_DRIVER__SRC__ROS_OUTPUT_HPP_
#define DM_SWERVE_DRIVER__SRC__ROS_OUTPUT_HPP_

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "dm_swerve_driver/control_types.hpp"
#include "dm_swerve_driver/params.hpp"

namespace dm_swerve_driver {

void publish_control_output(
  rclcpp_lifecycle::LifecycleNode & node,
  const DriverParameters & parameters,
  const ControlLoopOutput & output,
  const rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr &
  odometry_publisher,
  const rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr &
  joint_state_publisher,
  tf2_ros::TransformBroadcaster * tf_broadcaster);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__SRC__ROS_OUTPUT_HPP_
