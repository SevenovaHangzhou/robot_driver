#ifndef DM_SWERVE_DRIVER__IMU_VALIDATION_HPP_
#define DM_SWERVE_DRIVER__IMU_VALIDATION_HPP_

#include <sensor_msgs/msg/imu.hpp>

namespace dm_swerve_driver {

[[nodiscard]] bool imu_orientation_valid(
  const sensor_msgs::msg::Imu & message,
  double quaternion_norm_tolerance = 0.1) noexcept;

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__IMU_VALIDATION_HPP_
