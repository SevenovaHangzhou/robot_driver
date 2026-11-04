#ifndef LPMS_NAV3_CAN__ROS_CONVERSION_HPP_
#define LPMS_NAV3_CAN__ROS_CONVERSION_HPP_

#include "lpms_nav3_can/decoder.hpp"

#include <sensor_msgs/msg/imu.hpp>

#include <array>

namespace lpms_nav3_can
{

struct ImuCovariances
{
  std::array<double, 9> orientation{};
  std::array<double, 9> angular_velocity{};
  std::array<double, 9> linear_acceleration{};
};

[[nodiscard]] sensor_msgs::msg::Imu make_imu_message(
  const ImuSample & sample, const ImuCovariances & covariances, double mount_yaw_rad = 0.0);

}  // namespace lpms_nav3_can

#endif  // LPMS_NAV3_CAN__ROS_CONVERSION_HPP_
