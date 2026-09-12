#include "ros_output.hpp"

#include <array>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "dm_swerve_driver/odometry_covariance.hpp"

namespace dm_swerve_driver {
namespace {

const std::array<std::string, kKincoAxisCount> kJointNames{
  "front_left_steer_joint", "front_right_steer_joint",
  "rear_left_steer_joint", "rear_right_steer_joint",
  "front_left_wheel_joint", "front_right_wheel_joint",
  "rear_left_wheel_joint", "rear_right_wheel_joint"};

[[nodiscard]] geometry_msgs::msg::Quaternion yaw_quaternion(double yaw_rad)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw_rad);
  return tf2::toMsg(quaternion);
}

void publish_joint_states(
  const rclcpp::Time & stamp,
  const DriverParameters & parameters,
  const ControlLoopOutput & output,
  const rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr & publisher)
{
  sensor_msgs::msg::JointState joints;
  joints.header.stamp = stamp;
  joints.name.assign(kJointNames.begin(), kJointNames.end());
  joints.position.resize(kKincoAxisCount);
  joints.velocity.resize(kKincoAxisCount);
  for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
    joints.position[index] = output.steering_angle_rad[index];
    joints.position[index + kSwerveModuleCount] =
      output.wheel_distance_m[index] / parameters.chassis.wheel_radius_m;
    joints.velocity[index + kSwerveModuleCount] =
      output.wheel_velocity_mps[index] / parameters.chassis.wheel_radius_m;
  }
  publisher->publish(joints);
}

}  // namespace

void publish_control_output(
  rclcpp_lifecycle::LifecycleNode & node,
  const DriverParameters & parameters,
  const ControlLoopOutput & output,
  const rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr &
  odometry_publisher,
  const rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr &
  joint_state_publisher,
  tf2_ros::TransformBroadcaster * tf_broadcaster)
{
  if (!odometry_publisher || !odometry_publisher->is_activated()) {
    return;
  }
  const auto stamp = node.now();
  const auto orientation = yaw_quaternion(output.pose.heading_rad);
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = parameters.odometry.odom_frame;
  odometry.child_frame_id = parameters.odometry.base_frame;
  odometry.pose.pose.position.x = output.pose.x_m;
  odometry.pose.pose.position.y = output.pose.y_m;
  odometry.pose.pose.orientation = orientation;
  odometry.twist.twist.linear.x = output.measured_twist.vx_mps;
  odometry.twist.twist.linear.y = output.measured_twist.vy_mps;
  odometry.twist.twist.angular.z = output.measured_twist.omega_radps;
  const auto covariances = make_odometry_covariances(
    parameters.odometry, output.imu_fallback, output.valid_module_count,
    output.slip_detected);
  odometry.pose.covariance = covariances.pose;
  odometry.twist.covariance = covariances.twist;
  odometry_publisher->publish(odometry);
  publish_joint_states(stamp, parameters, output, joint_state_publisher);

  if (tf_broadcaster != nullptr) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parameters.odometry.odom_frame;
    transform.child_frame_id = parameters.odometry.base_frame;
    transform.transform.translation.x = output.pose.x_m;
    transform.transform.translation.y = output.pose.y_m;
    transform.transform.rotation = orientation;
    tf_broadcaster->sendTransform(transform);
  }
}

}  // namespace dm_swerve_driver
