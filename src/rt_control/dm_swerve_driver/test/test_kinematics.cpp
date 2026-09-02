#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "dm_swerve_driver/swerve_kinematics.hpp"
#include "dm_swerve_driver/swerve_odometry.hpp"

namespace dm_swerve_driver {
namespace {

constexpr double kTolerance{1e-12};

const std::array<Translation2d, kSwerveModuleCount> kLocations{
  Translation2d{0.5, 0.4}, Translation2d{0.5, -0.4},
  Translation2d{-0.5, 0.4}, Translation2d{-0.5, -0.4}};

[[nodiscard]] std::array<SwerveModulePosition, kSwerveModuleCount> positions_for_delta(
  double dx, double dy, double dtheta)
{
  std::array<SwerveModulePosition, kSwerveModuleCount> positions{};
  for (std::size_t index{0U}; index < positions.size(); ++index) {
    const double wheel_x{dx - dtheta * kLocations[index].y};
    const double wheel_y{dy + dtheta * kLocations[index].x};
    positions[index] = SwerveModulePosition{
      std::hypot(wheel_x, wheel_y), std::atan2(wheel_y, wheel_x), true};
  }
  return positions;
}

TEST(SwerveKinematicsTest, DiscretizeExponentiatesToRequestedFinitePose)
{
  constexpr double dt{0.02};
  const ChassisSpeeds requested{1.2, 0.4, kPi / 2.0};
  const ChassisSpeeds discrete{discretize(requested, dt)};
  const Pose2d actual{pose_exp(BodyDelta{
      discrete.vx_mps * dt, discrete.vy_mps * dt, discrete.omega_radps * dt})};

  EXPECT_NEAR(actual.x_m, requested.vx_mps * dt, kTolerance);
  EXPECT_NEAR(actual.y_m, requested.vy_mps * dt, kTolerance);
  EXPECT_NEAR(actual.heading_rad, requested.omega_radps * dt, kTolerance);
  EXPECT_LT(discrete.vy_mps, requested.vy_mps);
}

TEST(SwerveKinematicsTest, DiscretizeEliminatesVisibleEulerDrift)
{
  const ChassisSpeeds requested{1.0, 0.0, kPi / 2.0};
  const auto corrected = discretize(requested, 1.0);
  const auto corrected_pose = pose_exp(
    BodyDelta{corrected.vx_mps, corrected.vy_mps, corrected.omega_radps});
  const auto uncorrected_pose = pose_exp(BodyDelta{1.0, 0.0, kPi / 2.0});

  EXPECT_NEAR(corrected_pose.x_m, 1.0, kTolerance);
  EXPECT_NEAR(corrected_pose.y_m, 0.0, kTolerance);
  EXPECT_GT(std::hypot(uncorrected_pose.x_m - 1.0, uncorrected_pose.y_m), 0.7);
}

TEST(SwerveKinematicsTest, DiscretizeUsesCorrectCcwSignAndSmallAngleBranch)
{
  const auto turning = discretize(ChassisSpeeds{1.0, 0.0, 1.0}, 0.01);
  EXPECT_LT(turning.vy_mps, 0.0);
  EXPECT_DOUBLE_EQ(turning.omega_radps, 1.0);

  const auto tiny = discretize(ChassisSpeeds{1.0, -0.25, 1e-12}, 0.001);
  EXPECT_NEAR(tiny.vx_mps, 1.0, 1e-12);
  EXPECT_NEAR(tiny.vy_mps, -0.25, 1e-12);
}

TEST(SwerveKinematicsTest, InverseKinematicsHandlesTranslationAndRotation)
{
  const std::array<double, kSwerveModuleCount> previous{};
  const auto straight = inverse_kinematics(
    ChassisSpeeds{1.0, 0.0, 0.0}, kLocations, previous, 1e-3);
  for (const auto & state : straight) {
    EXPECT_NEAR(state.speed_mps, 1.0, kTolerance);
    EXPECT_NEAR(state.angle_rad, 0.0, kTolerance);
  }

  const auto spinning = inverse_kinematics(
    ChassisSpeeds{0.0, 0.0, 2.0}, kLocations, previous, 1e-3);
  for (std::size_t index{0U}; index < spinning.size(); ++index) {
    EXPECT_NEAR(spinning[index].speed_mps, 2.0 * std::hypot(0.5, 0.4), kTolerance);
    EXPECT_NEAR(
      spinning[index].angle_rad,
      std::atan2(2.0 * kLocations[index].x, -2.0 * kLocations[index].y),
      kTolerance);
  }
}

TEST(SwerveKinematicsTest, InverseKinematicsHoldsAngleBelowDeadband)
{
  const std::array<double, kSwerveModuleCount> previous{0.1, 0.2, 0.3, 0.4};
  const auto states = inverse_kinematics(
    ChassisSpeeds{1e-4, 0.0, 0.0}, kLocations, previous, 1e-3);
  for (std::size_t index{0U}; index < states.size(); ++index) {
    EXPECT_NEAR(states[index].angle_rad, previous[index], kTolerance);
  }
}

TEST(SwerveKinematicsTest, InverseKinematicsHandlesDiagonalTranslation)
{
  const std::array<double, kSwerveModuleCount> previous{};
  const auto diagonal = inverse_kinematics(
    ChassisSpeeds{1.0, 1.0, 0.0}, kLocations, previous, 1e-3);
  for (const auto & state : diagonal) {
    EXPECT_NEAR(state.speed_mps, std::sqrt(2.0), kTolerance);
    EXPECT_NEAR(state.angle_rad, kPi / 4.0, kTolerance);
  }
}

TEST(SwerveKinematicsTest, DesaturationPreservesRatios)
{
  std::array<SwerveModuleState, kSwerveModuleCount> states{
    SwerveModuleState{1.0, 0.0}, SwerveModuleState{2.0, 0.1},
    SwerveModuleState{3.0, 0.2}, SwerveModuleState{4.0, 0.3}};
  desaturate_wheel_speeds(states, 2.0);

  EXPECT_NEAR(states[0].speed_mps, 0.5, kTolerance);
  EXPECT_NEAR(states[1].speed_mps, 1.0, kTolerance);
  EXPECT_NEAR(states[2].speed_mps, 1.5, kTolerance);
  EXPECT_NEAR(states[3].speed_mps, 2.0, kTolerance);
}

TEST(SwerveKinematicsTest, OptimizeReversesSpeedAndKeepsContinuousTarget)
{
  const auto reversed = optimize_module(
    SwerveModuleState{2.0, 170.0 * kPi / 180.0}, 0.0);
  EXPECT_NEAR(reversed.speed_mps, -2.0, kTolerance);
  EXPECT_NEAR(reversed.error_rad, -10.0 * kPi / 180.0, kTolerance);
  EXPECT_NEAR(reversed.continuous_angle_rad, -10.0 * kPi / 180.0, kTolerance);

  const auto crossing = optimize_module(
    SwerveModuleState{1.0, -179.0 * kPi / 180.0}, 179.0 * kPi / 180.0);
  EXPECT_NEAR(crossing.error_rad, 2.0 * kPi / 180.0, kTolerance);
  EXPECT_NEAR(crossing.continuous_angle_rad, 181.0 * kPi / 180.0, kTolerance);
}

TEST(SwerveKinematicsTest, AlignmentAppliesCosineAndGatesAllWheels)
{
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  for (auto & state : desired) {
    state = SwerveModuleState{2.0, 10.0 * kPi / 180.0};
  }
  const std::array<double, kSwerveModuleCount> measured{};
  const auto aligned = optimize_and_apply_alignment(
    desired, measured, 20.0 * kPi / 180.0);
  EXPECT_FALSE(aligned.gated);
  for (const auto & state : aligned.modules) {
    EXPECT_NEAR(state.speed_mps, 2.0 * std::cos(10.0 * kPi / 180.0), kTolerance);
  }

  desired[2].angle_rad = 25.0 * kPi / 180.0;
  const auto gated = optimize_and_apply_alignment(
    desired, measured, 20.0 * kPi / 180.0);
  EXPECT_TRUE(gated.gated);
  EXPECT_NEAR(gated.maximum_error_rad, 25.0 * kPi / 180.0, kTolerance);
  for (const auto & state : gated.modules) {
    EXPECT_DOUBLE_EQ(state.speed_mps, 0.0);
  }
}

TEST(SwerveKinematicsTest, PositionDeltaForwardKinematicsUsesGyroYaw)
{
  constexpr double dx{0.12};
  constexpr double dy{-0.04};
  constexpr double dtheta{0.03};
  auto current = positions_for_delta(dx, dy, dtheta);
  auto previous = current;
  for (auto & position : previous) {
    position.distance_m = 0.0;
  }

  const auto translation = wheel_translation_from_position_deltas(
    previous, current, kLocations, dtheta);
  ASSERT_TRUE(translation.has_value());
  EXPECT_NEAR(translation->x, dx, kTolerance);
  EXPECT_NEAR(translation->y, dy, kTolerance);

  current[1].valid = false;
  const auto degraded = wheel_translation_from_position_deltas(
    previous, current, kLocations, dtheta);
  ASSERT_TRUE(degraded.has_value());
  EXPECT_NEAR(degraded->x, dx, kTolerance);
  EXPECT_NEAR(degraded->y, dy, kTolerance);
}

TEST(SwerveKinematicsTest, ForwardKinematicsUsesWrapSafeMidpointAndNoFeedbackIsEmpty)
{
  std::array<SwerveModulePosition, kSwerveModuleCount> previous{};
  std::array<SwerveModulePosition, kSwerveModuleCount> current{};
  for (std::size_t index{0U}; index < previous.size(); ++index) {
    previous[index] = SwerveModulePosition{0.0, 179.0 * kPi / 180.0, index == 0U};
    current[index] = SwerveModulePosition{1.0, -179.0 * kPi / 180.0, index == 0U};
  }

  const auto translation = wheel_translation_from_position_deltas(
    previous, current, kLocations, 0.0);
  ASSERT_TRUE(translation.has_value());
  EXPECT_NEAR(translation->x, -1.0, 2e-4);
  EXPECT_NEAR(translation->y, 0.0, 2e-4);

  current[0].valid = false;
  EXPECT_FALSE(wheel_translation_from_position_deltas(
      previous, current, kLocations, 0.0).has_value());
}

TEST(SwerveKinematicsTest, RejectsInvalidPhysicalLimits)
{
  std::array<SwerveModuleState, kSwerveModuleCount> states{};
  EXPECT_THROW(static_cast<void>(discretize(ChassisSpeeds{}, 0.0)), std::invalid_argument);
  EXPECT_THROW(desaturate_wheel_speeds(states, 0.0), std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(optimize_and_apply_alignment(states, {}, -0.1)),
    std::invalid_argument);
}

}  // namespace
}  // namespace dm_swerve_driver
