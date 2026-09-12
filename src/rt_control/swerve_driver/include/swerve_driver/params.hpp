#ifndef SWERVE_DRIVER__PARAMS_HPP_
#define SWERVE_DRIVER__PARAMS_HPP_

#include <array>
#include "swerve_driver/swerve_kinematics.hpp"

namespace swerve_driver
{
struct OdometryParameters
{
  std::array<double, 6> pose_covariance_diagonal{};
  std::array<double, 6> twist_covariance_diagonal{};
  double imu_fallback_covariance_scale{1.0};
  double missing_module_covariance_scale{1.0};
};
}  // namespace swerve_driver
#endif  // SWERVE_DRIVER__PARAMS_HPP_
