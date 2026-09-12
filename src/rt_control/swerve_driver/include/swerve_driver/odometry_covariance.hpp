#ifndef SWERVE_DRIVER__ODOMETRY_COVARIANCE_HPP_
#define SWERVE_DRIVER__ODOMETRY_COVARIANCE_HPP_

#include <array>
#include <cstddef>

#include "swerve_driver/params.hpp"

namespace swerve_driver {

struct OdometryCovariances {
  std::array<double, 36U> pose{};
  std::array<double, 36U> twist{};
};

[[nodiscard]] OdometryCovariances make_odometry_covariances(
  const OdometryParameters & parameters,
  bool imu_fallback,
  std::size_t valid_module_count);

}  // namespace swerve_driver

#endif  // SWERVE_DRIVER__ODOMETRY_COVARIANCE_HPP_
