#include "lpms_nav3_can/ros_conversion.hpp"

namespace lpms_nav3_can
{

sensor_msgs::msg::Imu make_imu_message(const ImuSample & sample)
{
  sensor_msgs::msg::Imu message;
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

sensor_msgs::msg::MagneticField make_magnetic_field_message(const ImuSample & sample)
{
  sensor_msgs::msg::MagneticField message;
  message.magnetic_field.x = sample.magnetic_field_t[0];
  message.magnetic_field.y = sample.magnetic_field_t[1];
  message.magnetic_field.z = sample.magnetic_field_t[2];
  return message;
}

}  // namespace lpms_nav3_can
