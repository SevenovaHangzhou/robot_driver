#include "periph_imu_lpms/ros_conversion.hpp"

#include <cmath>
#include <cstddef>

namespace periph_imu_lpms
{
namespace
{

std::array<double, 9> rotate_covariance(
  const std::array<double, 9> & covariance, const double cosine, const double sine)
{
  const std::array<double, 9> rotation = {cosine, -sine, 0.0, sine, cosine, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 9> rotated{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
          rotated[row * 3 + column] +=
            rotation[row * 3 + i] * covariance[i * 3 + j] * rotation[column * 3 + j];
        }
      }
    }
  }
  return rotated;
}

}  // namespace

sensor_msgs::msg::Imu make_imu_message(
  const ImuSample & sample, const ImuCovariances & covariances, const double mount_yaw_rad)
{
  sensor_msgs::msg::Imu message;
  const double cosine = std::cos(mount_yaw_rad);
  const double sine = std::sin(mount_yaw_rad);
  const double half_cosine = std::cos(mount_yaw_rad / 2.0);
  const double half_sine = std::sin(mount_yaw_rad / 2.0);
  const auto & q = sample.orientation_wxyz;
  // Change the reported sensor orientation to the robot-aligned output frame.
  message.orientation.w = q[0] * half_cosine + q[3] * half_sine;
  message.orientation.x = q[1] * half_cosine - q[2] * half_sine;
  message.orientation.y = q[1] * half_sine + q[2] * half_cosine;
  message.orientation.z = q[3] * half_cosine - q[0] * half_sine;
  message.angular_velocity.x =
    cosine * sample.angular_velocity_radps[0] - sine * sample.angular_velocity_radps[1];
  message.angular_velocity.y =
    sine * sample.angular_velocity_radps[0] + cosine * sample.angular_velocity_radps[1];
  message.angular_velocity.z = sample.angular_velocity_radps[2];
  message.linear_acceleration.x =
    cosine * sample.linear_acceleration_mps2[0] - sine * sample.linear_acceleration_mps2[1];
  message.linear_acceleration.y =
    sine * sample.linear_acceleration_mps2[0] + cosine * sample.linear_acceleration_mps2[1];
  message.linear_acceleration.z = sample.linear_acceleration_mps2[2];
  message.orientation_covariance = rotate_covariance(covariances.orientation, cosine, sine);
  message.angular_velocity_covariance =
    rotate_covariance(covariances.angular_velocity, cosine, sine);
  message.linear_acceleration_covariance =
    rotate_covariance(covariances.linear_acceleration, cosine, sine);
  return message;
}

}  // namespace periph_imu_lpms
