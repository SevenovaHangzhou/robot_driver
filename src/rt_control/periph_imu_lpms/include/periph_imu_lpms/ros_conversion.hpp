#ifndef PERIPH_IMU_LPMS__ROS_CONVERSION_HPP_
#define PERIPH_IMU_LPMS__ROS_CONVERSION_HPP_

#include "periph_imu_lpms/protocol/decoder.hpp"

#include <sensor_msgs/msg/imu.hpp>

#include <array>

namespace periph_imu_lpms
{

struct ImuCovariances
{
  std::array<double, 9> orientation{};
  std::array<double, 9> angular_velocity{};
  std::array<double, 9> linear_acceleration{};
};

[[nodiscard]] sensor_msgs::msg::Imu make_imu_message(
  const ImuSample & sample, const ImuCovariances & covariances, double mount_yaw_rad = 0.0);

}  // namespace periph_imu_lpms

#endif  // PERIPH_IMU_LPMS__ROS_CONVERSION_HPP_
