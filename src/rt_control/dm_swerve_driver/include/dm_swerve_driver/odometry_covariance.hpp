#ifndef DM_SWERVE_DRIVER__ODOMETRY_COVARIANCE_HPP_
#define DM_SWERVE_DRIVER__ODOMETRY_COVARIANCE_HPP_

#include <array>
#include <cstddef>

#include "dm_swerve_driver/params.hpp"

namespace dm_swerve_driver {

struct OdometryCovariances {
  std::array<double, 36U> pose{};
  std::array<double, 36U> twist{};
};

[[nodiscard]] OdometryCovariances make_odometry_covariances(
  const OdometryParameters & parameters,
  bool imu_fallback,
  std::size_t valid_module_count,
  bool slip_detected = false);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__ODOMETRY_COVARIANCE_HPP_
