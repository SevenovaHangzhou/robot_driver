#include "swerve_driver/odometry_covariance.hpp"

#include <cmath>
#include <stdexcept>

namespace swerve_driver {

OdometryCovariances make_odometry_covariances(
  const OdometryParameters & parameters,
  bool imu_fallback,
  std::size_t valid_module_count)
{
  if (valid_module_count > kSwerveModuleCount) {
    throw std::invalid_argument{"valid module count exceeds the swerve module count"};
  }
  double scale{1.0};
  if (imu_fallback) {
    scale *= parameters.imu_fallback_covariance_scale;
  }
  if (valid_module_count < kSwerveModuleCount) {
    scale *= parameters.missing_module_covariance_scale;
  }
  if (!std::isfinite(scale) || scale < 1.0) {
    throw std::invalid_argument{"odometry covariance scale must be finite and at least one"};
  }
  for (const double value : parameters.pose_covariance_diagonal) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument{"pose covariance diagonal must be finite and nonnegative"};
    }
  }
  for (const double value : parameters.twist_covariance_diagonal) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument{"twist covariance diagonal must be finite and nonnegative"};
    }
  }

  OdometryCovariances result;
  for (std::size_t index{0U}; index < 6U; ++index) {
    result.pose[index * 7U] = parameters.pose_covariance_diagonal[index] * scale;
    result.twist[index * 7U] = parameters.twist_covariance_diagonal[index] * scale;
  }
  return result;
}

}  // namespace swerve_driver
