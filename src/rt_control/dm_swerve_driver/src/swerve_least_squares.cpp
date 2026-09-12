#include "swerve_least_squares.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dm_swerve_driver {
namespace {

using AugmentedMatrix = std::array<std::array<double, 4U>, 3U>;

[[nodiscard]] std::optional<std::array<double, 3U>> solve_three_by_three(
  AugmentedMatrix matrix) noexcept
{
  constexpr double singular_tolerance{1e-12};
  for (std::size_t column{0U}; column < 3U; ++column) {
    std::size_t pivot{column};
    for (std::size_t row{column + 1U}; row < 3U; ++row) {
      if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
        pivot = row;
      }
    }
    if (std::abs(matrix[pivot][column]) < singular_tolerance) {
      return std::nullopt;
    }
    std::swap(matrix[column], matrix[pivot]);
    const double divisor{matrix[column][column]};
    for (std::size_t entry{column}; entry < 4U; ++entry) {
      matrix[column][entry] /= divisor;
    }
    for (std::size_t row{0U}; row < 3U; ++row) {
      if (row == column) {
        continue;
      }
      const double scale{matrix[row][column]};
      for (std::size_t entry{column}; entry < 4U; ++entry) {
        matrix[row][entry] -= scale * matrix[column][entry];
      }
    }
  }
  return std::array<double, 3U>{matrix[0][3], matrix[1][3], matrix[2][3]};
}

void accumulate_observation(
  AugmentedMatrix & normal,
  const std::array<double, 3U> & row,
  double measurement) noexcept
{
  for (std::size_t outer{0U}; outer < 3U; ++outer) {
    for (std::size_t inner{0U}; inner < 3U; ++inner) {
      normal[outer][inner] += row[outer] * row[inner];
    }
    normal[outer][3] += row[outer] * measurement;
  }
}

[[nodiscard]] std::optional<std::array<double, 3U>> solve_with_mask(
  const std::array<ModuleVectorObservation, kSwerveModuleCount> & observations,
  const std::array<Translation2d, kSwerveModuleCount> & locations,
  const std::array<bool, kSwerveModuleCount> & used) noexcept
{
  AugmentedMatrix normal{};
  for (std::size_t index{0U}; index < observations.size(); ++index) {
    if (!used[index]) {
      continue;
    }
    accumulate_observation(
      normal, {1.0, 0.0, -locations[index].y}, observations[index].x);
    accumulate_observation(
      normal, {0.0, 1.0, locations[index].x}, observations[index].y);
  }
  return solve_three_by_three(normal);
}

[[nodiscard]] double residual(
  const ModuleVectorObservation & observation,
  const Translation2d & location,
  const std::array<double, 3U> & solution) noexcept
{
  const double predicted_x{solution[0] - solution[2] * location.y};
  const double predicted_y{solution[1] + solution[2] * location.x};
  return std::hypot(observation.x - predicted_x, observation.y - predicted_y);
}

[[nodiscard]] std::size_t count_used(
  const std::array<bool, kSwerveModuleCount> & used) noexcept
{
  return static_cast<std::size_t>(std::count(used.begin(), used.end(), true));
}

}  // namespace

std::optional<ModuleLeastSquaresFit> fit_module_observations(
  const std::array<ModuleVectorObservation, kSwerveModuleCount> & observations,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations,
  const std::optional<double> & rejection_threshold)
{
  ModuleLeastSquaresFit result;
  for (std::size_t index{0U}; index < observations.size(); ++index) {
    result.used[index] = observations[index].valid;
  }

  while (count_used(result.used) >= 2U) {
    const auto solution = solve_with_mask(observations, module_locations, result.used);
    if (!solution.has_value()) {
      return std::nullopt;
    }
    result.solution = *solution;
    if (!rejection_threshold.has_value()) {
      break;
    }

    double largest_residual{-std::numeric_limits<double>::infinity()};
    std::size_t largest_index{0U};
    for (std::size_t index{0U}; index < observations.size(); ++index) {
      if (!result.used[index]) {
        continue;
      }
      const double value{residual(observations[index], module_locations[index], *solution)};
      if (value > largest_residual) {
        largest_residual = value;
        largest_index = index;
      }
    }
    if (largest_residual <= *rejection_threshold) {
      break;
    }
    if (count_used(result.used) == 2U) {
      return std::nullopt;
    }
    result.used[largest_index] = false;
    result.rejected[largest_index] = true;
  }

  result.used_count = count_used(result.used);
  if (result.used_count < 2U) {
    return std::nullopt;
  }
  for (std::size_t index{0U}; index < observations.size(); ++index) {
    if (observations[index].valid) {
      result.residuals[index] = residual(
        observations[index], module_locations[index], result.solution);
    }
  }
  return result;
}

}  // namespace dm_swerve_driver
