#ifndef PERIPH_IMU_LPMS__ROS_CONVERSION_HPP_
#define PERIPH_IMU_LPMS__ROS_CONVERSION_HPP_

#include <array>

#include "periph_imu_lpms/protocol/decoder.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace periph_imu_lpms
{

[[nodiscard]] sensor_msgs::msg::Imu make_imu_message(const ImuSample & sample);

[[nodiscard]] sensor_msgs::msg::Imu make_imu_message(
  const ImuSample & sample, const std::array<double, 9U> & orientation_covariance,
  const std::array<double, 9U> & angular_velocity_covariance);

[[nodiscard]] sensor_msgs::msg::MagneticField make_magnetic_field_message(const ImuSample & sample);

}  // namespace periph_imu_lpms

#endif  // PERIPH_IMU_LPMS__ROS_CONVERSION_HPP_
