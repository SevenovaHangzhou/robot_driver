#ifndef CTRL_SWERVE__PARAMS_HPP_
#define CTRL_SWERVE__PARAMS_HPP_

#include <array>
#include "ctrl_swerve/swerve_kinematics.hpp"

namespace ctrl_swerve
{
struct OdometryParameters
{
  std::array<double, 6> pose_covariance_diagonal{};
  std::array<double, 6> twist_covariance_diagonal{};
  double imu_fallback_covariance_scale{1.0};
  double missing_module_covariance_scale{1.0};
  double slip_covariance_scale{1.0};
};
}  // namespace ctrl_swerve
#endif  // CTRL_SWERVE__PARAMS_HPP_
