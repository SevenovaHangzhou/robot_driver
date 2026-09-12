#ifndef DM_SWERVE_DRIVER__SRC__SWERVE_LEAST_SQUARES_HPP_
#define DM_SWERVE_DRIVER__SRC__SWERVE_LEAST_SQUARES_HPP_

#include <array>
#include <cstddef>
#include <optional>

#include "dm_swerve_driver/swerve_kinematics.hpp"

namespace dm_swerve_driver {

struct ModuleVectorObservation {
  double x{0.0};
  double y{0.0};
  bool valid{false};
};

struct ModuleLeastSquaresFit {
  std::array<double, 3U> solution{};
  std::array<double, kSwerveModuleCount> residuals{};
  std::array<bool, kSwerveModuleCount> used{};
  std::array<bool, kSwerveModuleCount> rejected{};
  std::size_t used_count{0U};
};

[[nodiscard]] std::optional<ModuleLeastSquaresFit> fit_module_observations(
  const std::array<ModuleVectorObservation, kSwerveModuleCount> & observations,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations,
  const std::optional<double> & rejection_threshold);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__SRC__SWERVE_LEAST_SQUARES_HPP_
