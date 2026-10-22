#ifndef LPMS_NAV3_CAN__ROS_CONVERSION_HPP_
#define LPMS_NAV3_CAN__ROS_CONVERSION_HPP_

#include <array>

#include "lpms_nav3_can/protocol/decoder.hpp"

#include <sensor_msgs/msg/imu.hpp>

namespace lpms_nav3_can
{

[[nodiscard]] sensor_msgs::msg::Imu make_imu_message(const ImuSample & sample);

[[nodiscard]] sensor_msgs::msg::Imu make_imu_message(
  const ImuSample & sample,
  const std::array<double, 9U> & orientation_covariance,
  const std::array<double, 9U> & angular_velocity_covariance);

// Channels 7..9 (static-bias-calibrated gyro) as an Imu message carrying angular velocity
// only: orientation and linear acceleration are marked absent (covariance[0] = -1, REP-145).
[[nodiscard]] sensor_msgs::msg::Imu make_bias_corrected_gyro_message(const ImuSample & sample);

}  // namespace lpms_nav3_can

#endif  // LPMS_NAV3_CAN__ROS_CONVERSION_HPP_
