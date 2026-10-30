#ifndef CTRL_SWERVE__ODOMETRY_COVARIANCE_HPP_
#define CTRL_SWERVE__ODOMETRY_COVARIANCE_HPP_

#include <array>
#include <cstddef>

#include "ctrl_swerve/params.hpp"

namespace ctrl_swerve
{

struct OdometryCovariances
{
  std::array<double, 36U> pose{};
  std::array<double, 36U> twist{};
};

[[nodiscard]] OdometryCovariances make_odometry_covariances(
  const OdometryParameters & parameters, bool imu_fallback, std::size_t valid_module_count,
  bool slip_detected);

}  // namespace ctrl_swerve

#endif  // CTRL_SWERVE__ODOMETRY_COVARIANCE_HPP_
