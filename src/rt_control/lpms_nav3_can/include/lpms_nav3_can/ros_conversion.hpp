#ifndef LPMS_NAV3_CAN__ROS_CONVERSION_HPP_
#define LPMS_NAV3_CAN__ROS_CONVERSION_HPP_

#include <array>

#include "lpms_nav3_can/decoder.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace lpms_nav3_can
{

[[nodiscard]] sensor_msgs::msg::Imu make_imu_message(const ImuSample & sample);

[[nodiscard]] sensor_msgs::msg::Imu make_imu_message(
  const ImuSample & sample,
  const std::array<double, 9U> & orientation_covariance,
  const std::array<double, 9U> & angular_velocity_covariance);

[[nodiscard]] sensor_msgs::msg::MagneticField make_magnetic_field_message(
  const ImuSample & sample);

}  // namespace lpms_nav3_can

#endif  // LPMS_NAV3_CAN__ROS_CONVERSION_HPP_
