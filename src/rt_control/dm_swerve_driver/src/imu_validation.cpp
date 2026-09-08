#include "dm_swerve_driver/imu_validation.hpp"

#include <cmath>

namespace dm_swerve_driver {

bool imu_orientation_valid(
  const sensor_msgs::msg::Imu & message,
  double quaternion_norm_tolerance) noexcept
{
  if (!std::isfinite(quaternion_norm_tolerance) || quaternion_norm_tolerance < 0.0 ||
    !std::isfinite(message.orientation_covariance[0]) ||
    message.orientation_covariance[0] == -1.0)
  {
    return false;
  }
  const auto & orientation = message.orientation;
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
  {
    return false;
  }
  const double norm{std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w)};
  return std::abs(norm - 1.0) <= quaternion_norm_tolerance;
}

}  // namespace dm_swerve_driver
