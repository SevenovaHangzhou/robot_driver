#ifndef PERIPH_IMU_LPMS__ROS_CONVERSION_HPP_
#define PERIPH_IMU_LPMS__ROS_CONVERSION_HPP_

#include <array>

#include "periph_imu_lpms/protocol/decoder.hpp"

#include <sensor_msgs/msg/imu.hpp>

namespace periph_imu_lpms
{

[[nodiscard]] sensor_msgs::msg::Imu make_imu_message(const ImuSample & sample);

[[nodiscard]] sensor_msgs::msg::Imu make_imu_message(
  const ImuSample & sample, const std::array<double, 9U> & orientation_covariance,
  const std::array<double, 9U> & angular_velocity_covariance);

// Channels 7..9 (static-bias-calibrated gyro) as an Imu carrying angular velocity only:
// orientation and linear acceleration are marked absent (covariance[0] = -1).
[[nodiscard]] sensor_msgs::msg::Imu make_bias_corrected_gyro_message(const ImuSample & sample);

}  // namespace periph_imu_lpms

#endif  // PERIPH_IMU_LPMS__ROS_CONVERSION_HPP_
