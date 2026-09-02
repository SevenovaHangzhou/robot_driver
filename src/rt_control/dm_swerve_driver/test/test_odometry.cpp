#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "dm_swerve_driver/swerve_kinematics.hpp"
#include "dm_swerve_driver/swerve_odometry.hpp"

namespace dm_swerve_driver {
namespace {

constexpr double kTolerance{1e-11};

const std::array<Translation2d, kSwerveModuleCount> kLocations{
  Translation2d{0.5, 0.4}, Translation2d{0.5, -0.4},
  Translation2d{-0.5, 0.4}, Translation2d{-0.5, -0.4}};

[[nodiscard]] std::array<SwerveModulePosition, kSwerveModuleCount> initial_positions(
  double dx, double dy, double dtheta)
{
  std::array<SwerveModulePosition, kSwerveModuleCount> positions{};
  for (std::size_t index{0U}; index < positions.size(); ++index) {
    const double wheel_x{dx - dtheta * kLocations[index].y};
    const double wheel_y{dy + dtheta * kLocations[index].x};
    positions[index] = SwerveModulePosition{0.0, std::atan2(wheel_y, wheel_x), true};
  }
  return positions;
}

void advance_positions(
  std::array<SwerveModulePosition, kSwerveModuleCount> & positions,
  double dx, double dy, double dtheta)
{
  for (std::size_t index{0U}; index < positions.size(); ++index) {
    const double wheel_x{dx - dtheta * kLocations[index].y};
    const double wheel_y{dy + dtheta * kLocations[index].x};
    positions[index].distance_m += std::hypot(wheel_x, wheel_y);
    positions[index].angle_rad = std::atan2(wheel_y, wheel_x);
  }
}

TEST(SwerveOdometryMathTest, PoseExponentialHandlesStraightAndRotation)
{
  const auto straight = pose_exp(BodyDelta{1.5, -0.2, 0.0});
  EXPECT_DOUBLE_EQ(straight.x_m, 1.5);
  EXPECT_DOUBLE_EQ(straight.y_m, -0.2);
  EXPECT_DOUBLE_EQ(straight.heading_rad, 0.0);

  const auto arc = pose_exp(BodyDelta{1.0, 0.0, kPi / 2.0});
  EXPECT_NEAR(arc.x_m, 2.0 / kPi, kTolerance);
  EXPECT_NEAR(arc.y_m, 2.0 / kPi, kTolerance);
  EXPECT_NEAR(arc.heading_rad, kPi / 2.0, kTolerance);
}

TEST(SwerveOdometryMathTest, PoseLogAndExpRoundTrip)
{
  for (const BodyDelta delta : {
      BodyDelta{0.8, -0.3, 0.7},
      BodyDelta{-0.2, 0.5, -1.2},
      BodyDelta{1.0, 2.0, 1e-12}})
  {
    const auto recovered = pose_log(pose_exp(delta));
    EXPECT_NEAR(recovered.dx_m, delta.dx_m, kTolerance);
    EXPECT_NEAR(recovered.dy_m, delta.dy_m, kTolerance);
    EXPECT_NEAR(recovered.dtheta_rad, delta.dtheta_rad, kTolerance);
  }
}

TEST(SwerveOdometryMathTest, ExponentialMatchesClosedFormCircleAndEulerDoesNot)
{
  constexpr double radius{2.0};
  constexpr double angle{kPi / 2.0};
  const auto exact = pose_exp(BodyDelta{radius * angle, 0.0, angle});

  EXPECT_NEAR(exact.x_m, radius, kTolerance);
  EXPECT_NEAR(exact.y_m, radius, kTolerance);
  EXPECT_GT(std::hypot(radius * angle - exact.x_m, -exact.y_m), 2.0);
}

TEST(SwerveOdometryTest, IntegratesStraightPositionIncrements)
{
  auto positions = initial_positions(1.0, 0.0, 0.0);
  SwerveOdometry odometry{kLocations, 0.0, positions};
  advance_positions(positions, 1.0, 0.0, 0.0);

  const auto pose = odometry.update(0.0, positions);
  EXPECT_NEAR(pose.x_m, 1.0, kTolerance);
  EXPECT_NEAR(pose.y_m, 0.0, kTolerance);
  EXPECT_NEAR(pose.heading_rad, 0.0, kTolerance);
}

TEST(SwerveOdometryTest, PureGyroRotationProducesNoTranslation)
{
  constexpr double dtheta{0.2};
  auto positions = initial_positions(0.0, 0.0, dtheta);
  SwerveOdometry odometry{kLocations, 0.0, positions};
  advance_positions(positions, 0.0, 0.0, dtheta);

  const auto pose = odometry.update(dtheta, positions);
  EXPECT_NEAR(pose.x_m, 0.0, kTolerance);
  EXPECT_NEAR(pose.y_m, 0.0, kTolerance);
  EXPECT_NEAR(pose.heading_rad, dtheta, kTolerance);
}

TEST(SwerveOdometryTest, ConstantCurvatureLandsOnClosedFormArc)
{
  constexpr double radius{1.5};
  constexpr double dtheta{0.01};
  constexpr std::size_t steps{100U};
  auto positions = initial_positions(radius * dtheta, 0.0, dtheta);
  SwerveOdometry odometry{kLocations, 0.0, positions};

  for (std::size_t step{1U}; step <= steps; ++step) {
    advance_positions(positions, radius * dtheta, 0.0, dtheta);
    static_cast<void>(odometry.update(static_cast<double>(step) * dtheta, positions));
  }

  const double total_angle{static_cast<double>(steps) * dtheta};
  EXPECT_NEAR(odometry.pose().x_m, radius * std::sin(total_angle), 1e-10);
  EXPECT_NEAR(odometry.pose().y_m, radius * (1.0 - std::cos(total_angle)), 1e-10);
  EXPECT_NEAR(odometry.pose().heading_rad, total_angle, kTolerance);
}

TEST(SwerveOdometryTest, MissingModulesAreExcludedAndAllMissingStillTracksYaw)
{
  auto positions = initial_positions(0.5, 0.0, 0.0);
  SwerveOdometry odometry{kLocations, 0.0, positions};
  advance_positions(positions, 0.5, 0.0, 0.0);
  positions[2].valid = false;
  auto pose = odometry.update(0.0, positions);
  EXPECT_NEAR(pose.x_m, 0.5, kTolerance);

  for (auto & position : positions) {
    position.valid = false;
  }
  pose = odometry.update(0.2, positions);
  EXPECT_NEAR(pose.x_m, 0.5, kTolerance);
  EXPECT_NEAR(pose.y_m, 0.0, kTolerance);
  EXPECT_NEAR(pose.heading_rad, 0.2, kTolerance);
}

TEST(SwerveOdometryTest, ResetPreservesRequestedYawOffset)
{
  auto positions = initial_positions(0.0, 0.0, 0.0);
  SwerveOdometry odometry{kLocations, 0.2, positions, Pose2d{2.0, 3.0, 1.0}};
  auto pose = odometry.update(0.3, positions);
  EXPECT_NEAR(pose.heading_rad, 1.1, kTolerance);

  odometry.reset(Pose2d{-1.0, 4.0, -0.5}, 0.7, positions);
  pose = odometry.update(0.8, positions);
  EXPECT_NEAR(pose.x_m, -1.0, kTolerance);
  EXPECT_NEAR(pose.y_m, 4.0, kTolerance);
  EXPECT_NEAR(pose.heading_rad, -0.4, kTolerance);
}

}  // namespace
}  // namespace dm_swerve_driver
