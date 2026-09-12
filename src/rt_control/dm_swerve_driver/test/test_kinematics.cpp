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

TEST(SwerveKinematicsTest, OptimizeSelectsEquivalentTargetInsideMechanicalLimits)
{
  const auto reversed = optimize_module(
    SwerveModuleState{2.0, 170.0 * kPi / 180.0}, 0.0);
  EXPECT_NEAR(reversed.speed_mps, -2.0, kTolerance);
  EXPECT_NEAR(reversed.error_rad, -10.0 * kPi / 180.0, kTolerance);
  EXPECT_NEAR(reversed.target_angle_rad, -10.0 * kPi / 180.0, kTolerance);

  const auto crossing = optimize_module(
    SwerveModuleState{1.0, -179.0 * kPi / 180.0}, 179.0 * kPi / 180.0);
  EXPECT_NEAR(crossing.speed_mps, -1.0, kTolerance);
  EXPECT_NEAR(crossing.error_rad, -178.0 * kPi / 180.0, kTolerance);
  EXPECT_NEAR(crossing.target_angle_rad, 1.0 * kPi / 180.0, kTolerance);
}

TEST(SwerveKinematicsTest, AlignmentAppliesCosineAndGatesAllWheels)
{
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  for (auto & state : desired) {
    state = SwerveModuleState{2.0, 10.0 * kPi / 180.0};
  }
  const std::array<double, kSwerveModuleCount> measured{};
  SwerveSetpointGenerator aligned_generator{SwerveSetpointParameters{
      20.0 * kPi / 180.0, 0.0, 1000.0, {}}};
  const auto aligned = aligned_generator.generate(desired, measured, 0.01);
  EXPECT_FALSE(aligned.gated);
  for (const auto & state : aligned.modules) {
    EXPECT_NEAR(state.speed_mps, 2.0 * std::cos(10.0 * kPi / 180.0), kTolerance);
  }

  desired[2].angle_rad = 25.0 * kPi / 180.0;
  SwerveSetpointGenerator gated_generator{SwerveSetpointParameters{
      20.0 * kPi / 180.0, 0.0, 1000.0, {}}};
  const auto gated = gated_generator.generate(desired, measured, 0.01);
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
    static_cast<void>(SwerveSetpointGenerator(
        SwerveSetpointParameters{-0.1, 0.1, 1.0, {}})),
    std::invalid_argument);
}

TEST(SwerveKinematicsTest, MeasuredModuleStatesRecoverChassisSpeeds)
{
  const ChassisSpeeds expected{0.8, -0.3, 0.7};
  const std::array<double, kSwerveModuleCount> previous{};
  const auto states = inverse_kinematics(expected, kLocations, previous, 0.0);
  std::array<SwerveModuleMeasurement, kSwerveModuleCount> measured{};
  for (std::size_t index{0U}; index < measured.size(); ++index) {
    measured[index] = SwerveModuleMeasurement{
      states[index].speed_mps, states[index].angle_rad, true};
  }
  measured[2].valid = false;

  const auto actual = chassis_speeds_from_module_states(measured, kLocations);
  ASSERT_TRUE(actual.has_value());
  EXPECT_NEAR(actual->vx_mps, expected.vx_mps, kTolerance);
  EXPECT_NEAR(actual->vy_mps, expected.vy_mps, kTolerance);
  EXPECT_NEAR(actual->omega_radps, expected.omega_radps, kTolerance);
}

TEST(SwerveKinematicsTest, MeasuredChassisSpeedsNeedAtLeastTwoModules)
{
  std::array<SwerveModuleMeasurement, kSwerveModuleCount> measured{};
  measured[0] = SwerveModuleMeasurement{1.0, 0.0, true};
  for (std::size_t index{1U}; index < measured.size(); ++index) {
    measured[index].valid = false;
  }
  EXPECT_FALSE(chassis_speeds_from_module_states(measured, kLocations).has_value());
}

TEST(SwerveKinematicsTest, ResidualFitLocatesAndRejectsSingleWheelSlip)
{
  const ChassisSpeeds expected{0.8, -0.3, 0.7};
  const std::array<double, kSwerveModuleCount> previous{};
  const auto states = inverse_kinematics(expected, kLocations, previous, 0.0);
  std::array<SwerveModuleMeasurement, kSwerveModuleCount> measured{};
  for (std::size_t index{0U}; index < measured.size(); ++index) {
    measured[index] = SwerveModuleMeasurement{
      states[index].speed_mps, states[index].angle_rad, true};
  }
  measured[2].speed_mps += 2.0;

  const auto fit = chassis_speeds_with_slip_rejection(measured, kLocations, 0.1);

  ASSERT_TRUE(fit.has_value());
  EXPECT_TRUE(fit->slipping_modules[2]);
  EXPECT_FALSE(fit->used_modules[2]);
  EXPECT_EQ(fit->used_module_count, 3U);
  EXPECT_NEAR(fit->speeds.vx_mps, expected.vx_mps, kTolerance);
  EXPECT_NEAR(fit->speeds.vy_mps, expected.vy_mps, kTolerance);
  EXPECT_NEAR(fit->speeds.omega_radps, expected.omega_radps, kTolerance);
  EXPECT_GT(fit->residual_mps[2], 0.1);
}

TEST(SwerveKinematicsTest, ResidualFitDocumentsCommonModeBlindSpot)
{
  const ChassisSpeeds common_mode_motion{1.3, 0.2, -0.4};
  const std::array<double, kSwerveModuleCount> previous{};
  const auto states = inverse_kinematics(common_mode_motion, kLocations, previous, 0.0);
  std::array<SwerveModuleMeasurement, kSwerveModuleCount> measured{};
  for (std::size_t index{0U}; index < measured.size(); ++index) {
    measured[index] = SwerveModuleMeasurement{
      states[index].speed_mps, states[index].angle_rad, true};
  }

  const auto fit = chassis_speeds_with_slip_rejection(measured, kLocations, 0.01);

  ASSERT_TRUE(fit.has_value());
  EXPECT_FALSE(fit->slip_detected());
  EXPECT_EQ(fit->used_module_count, kSwerveModuleCount);
  EXPECT_NEAR(fit->speeds.vx_mps, common_mode_motion.vx_mps, kTolerance);
  EXPECT_NEAR(fit->speeds.vy_mps, common_mode_motion.vy_mps, kTolerance);
  EXPECT_NEAR(fit->speeds.omega_radps, common_mode_motion.omega_radps, kTolerance);
}

TEST(SwerveKinematicsTest, ResidualFitRejectsInvalidThreshold)
{
  std::array<SwerveModuleMeasurement, kSwerveModuleCount> measured{};
  EXPECT_THROW(
    static_cast<void>(chassis_speeds_with_slip_rejection(measured, kLocations, 0.0)),
    std::invalid_argument);
}

TEST(SwerveKinematicsTest, FlipSelectionUsesHysteresisAroundNinetyDegrees)
{
  constexpr double hysteresis{0.1};
  bool reversed{false};
  auto state = optimize_module(
    SwerveModuleState{1.0, 92.0 * kPi / 180.0}, 0.0, reversed, hysteresis);
  EXPECT_FALSE(state.reversed);
  EXPECT_GT(state.speed_mps, 0.0);

  state = optimize_module(
    SwerveModuleState{1.0, 97.0 * kPi / 180.0}, 0.0,
    state.reversed, hysteresis);
  EXPECT_TRUE(state.reversed);
  EXPECT_LT(state.speed_mps, 0.0);

  state = optimize_module(
    SwerveModuleState{1.0, 88.0 * kPi / 180.0}, 0.0,
    state.reversed, hysteresis);
  EXPECT_TRUE(state.reversed);
  EXPECT_LT(state.speed_mps, 0.0);

  state = optimize_module(
    SwerveModuleState{1.0, 80.0 * kPi / 180.0}, 0.0,
    state.reversed, hysteresis);
  EXPECT_FALSE(state.reversed);
  EXPECT_GT(state.speed_mps, 0.0);
}

TEST(SwerveKinematicsTest, SteeringSlewPrecedesAlignmentGate)
{
  SwerveSetpointGenerator generator{
    SwerveSetpointParameters{0.15, 0.1, 1.0}};
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  std::array<double, kSwerveModuleCount> measured{};
  for (auto & module : desired) {
    module = SwerveModuleState{1.0, 1.0};
  }

  const auto setpoint = generator.generate(desired, measured, 0.1);
  EXPECT_NEAR(setpoint.maximum_error_rad, 1.0, kTolerance);
  EXPECT_TRUE(setpoint.gated);
  for (const auto & module : setpoint.modules) {
    EXPECT_NEAR(module.target_angle_rad, 0.1, kTolerance);
    EXPECT_NEAR(module.error_rad, 1.0, kTolerance);
    EXPECT_DOUBLE_EQ(module.speed_mps, 0.0);
  }
}

TEST(SwerveKinematicsTest, SteeringSlewStaysInsideLimitsNearPiBoundary)
{
  SwerveSetpointGenerator generator{
    SwerveSetpointParameters{kPi, 0.1, 1.0}};
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  std::array<double, kSwerveModuleCount> measured{};
  desired.fill(SwerveModuleState{1.0, -179.0 * kPi / 180.0});
  measured.fill(179.0 * kPi / 180.0);

  for (int cycle{0}; cycle < 40; ++cycle) {
    const auto setpoint = generator.generate(desired, measured, 0.1);
    for (std::size_t index{0U}; index < setpoint.modules.size(); ++index) {
      EXPECT_TRUE(setpoint.modules[index].reversed);
      EXPECT_GE(setpoint.modules[index].target_angle_rad, -kPi);
      EXPECT_LE(setpoint.modules[index].target_angle_rad, kPi);
      measured[index] = setpoint.modules[index].target_angle_rad;
    }
  }
  for (const auto angle : measured) {
    EXPECT_NEAR(angle, 1.0 * kPi / 180.0, kTolerance);
  }
}

TEST(SwerveKinematicsTest, SteeringLimitsMustCoverEveryWheelHeading)
{
  EXPECT_THROW(
    SwerveSetpointGenerator(SwerveSetpointParameters{
        0.2, 0.1, 1.0, SteeringAngleLimits{-0.5, 0.5}}),
    std::invalid_argument);
}

TEST(SwerveKinematicsTest, BoundedOptimizationCanSelectPositivePiEndpoint)
{
  const auto state = optimize_module(
    SwerveModuleState{1.0, 0.0},
    179.0 * kPi / 180.0,
    false,
    0.0,
    SteeringAngleLimits{-kPi, kPi, 0.0});

  EXPECT_TRUE(state.reversed);
  EXPECT_NEAR(state.target_angle_rad, kPi, kTolerance);
  EXPECT_NEAR(state.error_rad, kPi / 180.0, kTolerance);
  EXPECT_NEAR(state.speed_mps, -1.0, kTolerance);
}

TEST(SwerveKinematicsTest, RearLeftPiCrossingUsesLinearErrorAndGatesAllDrive)
{
  SwerveSetpointGenerator generator{SwerveSetpointParameters{
      20.0 * kPi / 180.0, 0.1, 1000.0, {}}};
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  std::array<double, kSwerveModuleCount> measured{};
  desired.fill(SwerveModuleState{1.0, 0.0});
  desired[2].angle_rad = -179.0 * kPi / 180.0;
  measured[2] = 179.0 * kPi / 180.0;

  const auto setpoint = generator.generate(desired, measured, 0.01);

  EXPECT_TRUE(setpoint.gated);
  EXPECT_TRUE(setpoint.modules[2].reversed);
  EXPECT_NEAR(setpoint.modules[2].target_angle_rad, kPi / 180.0, kTolerance);
  EXPECT_NEAR(setpoint.modules[2].error_rad, -178.0 * kPi / 180.0, kTolerance);
  for (const auto & module : setpoint.modules) {
    EXPECT_DOUBLE_EQ(module.speed_mps, 0.0);
  }
}

TEST(SwerveKinematicsTest, BoundedOptimizationGridNeverLeavesSafeInterval)
{
  constexpr SteeringAngleLimits limits{-kPi, kPi, 0.05};
  constexpr int samples{72};
  const double safe_minimum{limits.minimum_rad + limits.margin_rad};
  const double safe_maximum{limits.maximum_rad - limits.margin_rad};
  for (int current_index{0}; current_index <= samples; ++current_index) {
    const double current = safe_minimum +
      (safe_maximum - safe_minimum) * static_cast<double>(current_index) / samples;
    for (int desired_index{0}; desired_index <= samples; ++desired_index) {
      const double desired{-kPi + 2.0 * kPi * desired_index / samples};
      for (const bool previous_reversed : {false, true}) {
        const auto state = optimize_module(
          SwerveModuleState{1.0, desired}, current,
          previous_reversed, 0.1, limits);
        EXPECT_GE(state.target_angle_rad, safe_minimum);
        EXPECT_LE(state.target_angle_rad, safe_maximum);
        EXPECT_NEAR(
          state.error_rad, state.target_angle_rad - current, kTolerance);
      }
    }
  }
}

TEST(SwerveKinematicsTest, MeasurementsNearPhysicalLimitsAreClampedIntoSafeRange)
{
  constexpr SteeringAngleLimits limits{-kPi, kPi, 0.1, 0.02};
  SwerveSetpointGenerator generator{
    SwerveSetpointParameters{kPi, 0.1, 1.0, limits}};
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  desired.fill(SwerveModuleState{1.0, 0.0});
  std::array<double, kSwerveModuleCount> measured{};
  measured.fill(kPi - 0.05);

  auto setpoint = generator.generate(desired, measured, 0.1);
  for (const auto & module : setpoint.modules) {
    EXPECT_LE(module.target_angle_rad, kPi - limits.margin_rad);
    EXPECT_GE(module.target_angle_rad, -kPi + limits.margin_rad);
  }

  SwerveSetpointGenerator tolerance_generator{
    SwerveSetpointParameters{kPi, 0.1, 1.0, limits}};
  measured.fill(kPi + 0.01);
  EXPECT_NO_THROW(setpoint = tolerance_generator.generate(desired, measured, 0.1));
  for (const auto & module : setpoint.modules) {
    EXPECT_LE(module.target_angle_rad, kPi - limits.margin_rad);
  }
}

}  // namespace
}  // namespace dm_swerve_driver
