#include "lpms_nav3_can/ros_conversion.hpp"

namespace lpms_nav3_can
{

sensor_msgs::msg::Imu make_imu_message(const ImuSample & sample)
{
  return make_imu_message(sample, {}, {});
}

sensor_msgs::msg::Imu make_imu_message(
  const ImuSample & sample,
  const std::array<double, 9U> & orientation_covariance,
  const std::array<double, 9U> & angular_velocity_covariance)
{
  sensor_msgs::msg::Imu message;
  message.orientation_covariance = orientation_covariance;
  message.angular_velocity_covariance = angular_velocity_covariance;
  message.orientation.w = sample.orientation_wxyz[0];
  message.orientation.x = sample.orientation_wxyz[1];
  message.orientation.y = sample.orientation_wxyz[2];
  message.orientation.z = sample.orientation_wxyz[3];
  message.angular_velocity.x = sample.angular_velocity_radps[0];
  message.angular_velocity.y = sample.angular_velocity_radps[1];
  message.angular_velocity.z = sample.angular_velocity_radps[2];
  message.linear_acceleration.x = sample.linear_acceleration_mps2[0];
  message.linear_acceleration.y = sample.linear_acceleration_mps2[1];
  message.linear_acceleration.z = sample.linear_acceleration_mps2[2];
  return message;
}

sensor_msgs::msg::Imu make_bias_corrected_gyro_message(const ImuSample & sample)
{
  sensor_msgs::msg::Imu message;
  message.orientation_covariance[0] = -1.0;
  message.linear_acceleration_covariance[0] = -1.0;
  message.angular_velocity.x = sample.angular_velocity_bias_corrected_radps[0];
  message.angular_velocity.y = sample.angular_velocity_bias_corrected_radps[1];
  message.angular_velocity.z = sample.angular_velocity_bias_corrected_radps[2];
  return message;
}

}  // namespace lpms_nav3_can
