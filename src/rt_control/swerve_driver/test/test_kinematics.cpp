#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "swerve_driver/body_velocity_shaper.hpp"
#include "swerve_driver/swerve_kinematics.hpp"
#include "swerve_driver/swerve_odometry.hpp"

namespace swerve_driver {
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
  EXPECT_TRUE(crossing.reversed);
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

TEST(SwerveKinematicsTest, ResidualFitRejectsOneInconsistentWheel)
{
  const ChassisSpeeds expected{0.8, -0.3, 0.7};
  const auto states = inverse_kinematics(expected, kLocations, {}, 0.0);
  std::array<SwerveModuleMeasurement, kSwerveModuleCount> measured{};
  for (std::size_t index{0U}; index < measured.size(); ++index) {
    measured[index] = {states[index].speed_mps, states[index].angle_rad, true};
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
  const ChassisSpeeds consistent_but_wrong{1.3, 0.2, -0.4};
  const auto states = inverse_kinematics(consistent_but_wrong, kLocations, {}, 0.0);
  std::array<SwerveModuleMeasurement, kSwerveModuleCount> measured{};
  for (std::size_t index{0U}; index < measured.size(); ++index) {
    measured[index] = {states[index].speed_mps, states[index].angle_rad, true};
  }

  const auto fit = chassis_speeds_with_slip_rejection(measured, kLocations, 0.01);

  ASSERT_TRUE(fit.has_value());
  EXPECT_FALSE(fit->slip_detected());
  EXPECT_EQ(fit->used_module_count, kSwerveModuleCount);
  EXPECT_NEAR(fit->speeds.vx_mps, consistent_but_wrong.vx_mps, kTolerance);
  EXPECT_NEAR(fit->speeds.vy_mps, consistent_but_wrong.vy_mps, kTolerance);
  EXPECT_NEAR(fit->speeds.omega_radps, consistent_but_wrong.omega_radps, kTolerance);
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

TEST(SwerveKinematicsTest, AlignFirstGateHoldsAllWheelsUntilRelease)
{
  constexpr double kDeg{kPi / 180.0};
  SwerveSetpointParameters parameters{};
  parameters.alignment_threshold_rad = 5.0 * kDeg;
  parameters.alignment_release_rad = 3.0 * kDeg;
  parameters.maximum_steering_slew_radps = 10.0;
  SwerveSetpointGenerator generator{parameters};
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  desired.fill(SwerveModuleState{1.0, 0.0});
  std::array<double, kSwerveModuleCount> measured{};
  auto step = [&](double module0_error_deg) {
    measured.fill(0.0);
    measured[0] = -module0_error_deg * kDeg;  // only module 1 lags
    return generator.generate(desired, measured, 0.001);
  };
  EXPECT_FALSE(step(4.0).gated);   // below the 5 deg entry: drive (cos-scaled)
  const auto entered = step(6.0);  // one module above 5 deg stops all four wheels
  EXPECT_TRUE(entered.gated);
  for (const auto & module : entered.modules) {
    EXPECT_DOUBLE_EQ(module.speed_mps, 0.0);
  }
  EXPECT_TRUE(step(4.0).gated);                   // inside the hysteresis band: still waiting
  EXPECT_FALSE(step(2.0).gated);                  // every module within 3 deg: wheels restart
  parameters.alignment_release_rad = 6.0 * kDeg;  // release above entry is rejected
  EXPECT_THROW(SwerveSetpointGenerator{parameters}, std::invalid_argument);
}

TEST(SwerveKinematicsTest, BodyShaperKeepsDirectionAndLimits)
{
  BodyShaperLimits limits;
  limits.acceleration = 1.0;
  limits.jerk = 4.0;
  limits.rotation_radius_m = 0.5;
  BodyVelocityShaper shaper{limits};
  const ChassisSpeeds target{0.2, 0.1, 0.4};  // omega * R = 0.2
  constexpr double dt{0.001};
  std::array<double, 3> previous_a{};
  double previous_speed{0.0};
  ChassisSpeeds shaped{};
  for (int k = 0; k < 2000; ++k) {
    shaped = shaper.step(target, dt);
    // Collinear with the target: vx : vy : omega stay fixed, so module angles stay fixed.
    EXPECT_NEAR(shaped.vy_mps * target.vx_mps, shaped.vx_mps * target.vy_mps, 1e-12);
    EXPECT_NEAR(shaped.omega_radps * target.vx_mps, shaped.vx_mps * target.omega_radps, 1e-12);
    const auto & a = shaper.acceleration();
    EXPECT_LE(std::hypot(a[0], a[1], a[2]), limits.acceleration + 1e-9);
    EXPECT_TRUE(shaper.valid());
    EXPECT_LE(
      std::hypot(a[0] - previous_a[0], a[1] - previous_a[1], a[2] - previous_a[2]),
      limits.jerk * dt + 1e-9);
    const double speed = std::hypot(shaped.vx_mps, shaped.vy_mps, shaped.omega_radps * 0.5);
    EXPECT_GE(speed, previous_speed - 1e-12);  // monotonic, no overshoot
    EXPECT_LE(speed, std::hypot(0.2, 0.1, 0.2) + 1e-12);
    previous_speed = speed;
    previous_a = a;
  }
  EXPECT_NEAR(shaped.vx_mps, 0.2, 1e-9);
  EXPECT_NEAR(shaped.omega_radps, 0.4, 1e-9);
  for (int k = 0; k < 2000; ++k) {
    shaped = shaper.step(ChassisSpeeds{}, dt);
    EXPECT_NEAR(shaped.vy_mps * target.vx_mps, shaped.vx_mps * target.vy_mps, 1e-12);
  }
  EXPECT_NEAR(std::hypot(shaped.vx_mps, shaped.vy_mps, shaped.omega_radps), 0.0, 1e-12);
  // Small final corrections also start from zero without a seed-speed step.
  shaper.reset();
  shaped = shaper.step(ChassisSpeeds{0.0, 0.0, 0.006}, dt);
  EXPECT_GT(shaped.omega_radps, 0.0);
  EXPECT_LE(shaped.omega_radps * 0.5, 0.5 * limits.jerk * dt * dt);
  for (int k = 0; k < 1000; ++k) {
    shaped = shaper.step(ChassisSpeeds{0.0, 0.0, 0.006}, dt);
  }
  EXPECT_NEAR(shaped.omega_radps, 0.006, 1e-12);
  // Reversal (+0.3 -> -0.3 m/s) passes through zero without an acceleration step.
  shaper.reset();
  for (int k = 0; k < 2000; ++k) {
    (void)shaper.step(ChassisSpeeds{0.3, 0.0, 0.0}, dt);
  }
  previous_a = shaper.acceleration();
  for (int k = 0; k < 3000; ++k) {
    shaped = shaper.step(ChassisSpeeds{-0.3, 0.0, 0.0}, dt);
    const auto & a = shaper.acceleration();
    EXPECT_LE(std::abs(a[0] - previous_a[0]), limits.jerk * dt + 1e-9);
    previous_a = a;
  }
  EXPECT_NEAR(shaped.vx_mps, -0.3, 1e-9);
  // Continuous retargeting, including zero commands, at typical command update rates.
  previous_a = shaper.acceleration();
  for (int k = 0; k < 4000; ++k) {
    const double phase = static_cast<double>(k / 40) * 0.17;
    const ChassisSpeeds moving =
      k % 500 < 100
        ? ChassisSpeeds{}
        : ChassisSpeeds{0.3 * std::sin(phase), 0.2 * std::cos(phase), 0.4 * std::sin(2 * phase)};
    const double period = k % 2 == 0 ? 0.0008 : 0.0012;
    shaped = shaper.step(moving, period);
    ASSERT_TRUE(shaper.valid());
    const auto & a = shaper.acceleration();
    EXPECT_LE(std::hypot(a[0], a[1], a[2]), limits.acceleration + 1e-9);
    EXPECT_LE(
      std::hypot(a[0] - previous_a[0], a[1] - previous_a[1], a[2] - previous_a[2]),
      limits.jerk * period + 1e-8);
    previous_a = a;
  }
  (void)shaper.step({std::numeric_limits<double>::quiet_NaN(), 0, 0}, dt);
  EXPECT_FALSE(shaper.valid());
  shaper.reset();
  EXPECT_TRUE(shaper.valid());
  EXPECT_THROW((BodyVelocityShaper{BodyShaperLimits{0.0}}), std::invalid_argument);
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
  SwerveSetpointParameters parameters{0.2, 0.1, 1.0};
  parameters.angle_limits[2] = SteeringAngleLimits{-0.5, 0.5};
  EXPECT_THROW(
    {SwerveSetpointGenerator generator{parameters};},
    std::invalid_argument);
}

TEST(SwerveKinematicsTest, TranslationUsesOneFeasibleBranchForAllModules)
{
  SwerveSetpointParameters parameters{kPi, 0.1, 1000.0};
  parameters.translation_heading_epsilon_rad = 1e-6;
  SwerveSetpointGenerator generator{parameters};
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  desired.fill(SwerveModuleState{1.0, 0.0});
  std::array<double, kSwerveModuleCount> measured{
    170.0 * kPi / 180.0, 170.0 * kPi / 180.0,
    170.0 * kPi / 180.0, 0.0};

  const auto setpoint = generator.generate(desired, measured, 0.01);

  for (const auto & module : setpoint.modules) {
    EXPECT_TRUE(module.reversed);
    EXPECT_GE(module.target_angle_rad, -kPi);
    EXPECT_LE(module.target_angle_rad, kPi);
  }
}

TEST(SwerveKinematicsTest, SteeringDeadbandHoldsPreviousSafeTarget)
{
  SwerveSetpointParameters parameters{kPi, 0.1, 1000.0};
  parameters.steering_angle_deadband_rad = 0.02;
  SwerveSetpointGenerator generator{parameters};
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  desired.fill(SwerveModuleState{1.0, 0.01});

  const auto setpoint = generator.generate(desired, {}, 0.01);

  for (const auto & module : setpoint.modules) {
    EXPECT_DOUBLE_EQ(module.target_angle_rad, 0.0);
  }
}

TEST(SwerveKinematicsTest, MeasurementToleranceNeverExpandsCommandRange)
{
  SwerveSetpointParameters parameters{kPi, 0.1, 1.0};
  parameters.angle_limits.fill(SteeringAngleLimits{-kPi, kPi, 0.1, 0.02});
  SwerveSetpointGenerator generator{parameters};
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  desired.fill(SwerveModuleState{1.0, 0.0});
  std::array<double, kSwerveModuleCount> measured{};
  measured.fill(kPi + 0.01);

  const auto setpoint = generator.generate(desired, measured, 0.1);

  for (const auto & module : setpoint.modules) {
    EXPECT_LE(module.target_angle_rad, kPi - 0.1);
    EXPECT_GE(module.target_angle_rad, -kPi + 0.1);
  }
  measured[0] = kPi + 0.03;
  EXPECT_THROW(
    static_cast<void>(generator.generate(desired, measured, 0.1)),
    std::out_of_range);
}

}  // namespace
}  // namespace swerve_driver

namespace swerve_driver
{
TEST(SwerveSetpointGeneratorTest, SteeringRateRampsUpAndRampsDownBeforeLatching)
{
  SwerveSetpointParameters parameters{kPi, 0.1, 0.6};
  parameters.maximum_steering_acceleration_radps2 = 0.6;
  SwerveSetpointGenerator generator{parameters};
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  std::array<double, kSwerveModuleCount> measured{};
  desired.fill(SwerveModuleState{1.0, 1.0});
  constexpr double dt{0.004};
  double previous_target{0.0};
  double previous_rate{0.0};
  for (int cycle{0}; cycle < 400; ++cycle) {
    const auto setpoint = generator.generate(desired, measured, dt);
    const double rate{(setpoint.modules[0].target_angle_rad - previous_target) / dt};
    EXPECT_LE(rate, 0.6 + 1e-9);
    EXPECT_LE(std::abs(rate - previous_rate), 0.6 * dt + 1e-9);
    previous_rate = rate;
    previous_target = setpoint.modules[0].target_angle_rad;
  }
  EXPECT_GT(previous_rate, 0.5);
  // Holding keeps decelerating at the same limit and then latches the reached target.
  std::array<bool, kSwerveModuleCount> holding{};
  holding.fill(true);
  for (int cycle{0}; cycle < 400; ++cycle) {
    const auto setpoint = generator.generate(desired, measured, dt, holding);
    const double rate{(setpoint.modules[0].target_angle_rad - previous_target) / dt};
    EXPECT_GE(rate, -1e-12);
    EXPECT_LE(previous_rate - rate, 0.6 * dt + 1e-9);
    previous_rate = rate;
    previous_target = setpoint.modules[0].target_angle_rad;
  }
  EXPECT_DOUBLE_EQ(previous_rate, 0.0);
  const auto latched = generator.generate(desired, measured, dt, holding);
  EXPECT_DOUBLE_EQ(latched.modules[0].target_angle_rad, previous_target);
}
}  // namespace swerve_driver

namespace swerve_driver
{
namespace
{
// Drives one generator towards ``goal`` for ``steps`` 1 ms cycles with perfect tracking and
// returns the commanded angle history of module 0.
std::vector<double> run_steering(
  SwerveSetpointGenerator & generator, std::array<double, kSwerveModuleCount> & angles, double goal,
  int steps)
{
  std::vector<double> history;
  for (int step = 0; step < steps; ++step) {
    std::array<SwerveModuleState, kSwerveModuleCount> desired{};
    for (auto & state : desired) {
      state = {0.5, goal};
    }
    const auto result = generator.generate(desired, angles, 0.001);
    for (auto & angle : angles) {
      angle = result.modules[0].target_angle_rad;
    }
    history.push_back(angles[0]);
  }
  return history;
}

SwerveSetpointParameters s_curve_parameters()
{
  SwerveSetpointParameters parameters{kPi, 0.1, 35.0 * kPi / 180.0};
  parameters.maximum_steering_acceleration_radps2 = 20.0 * kPi / 180.0;
  parameters.maximum_steering_jerk_radps3 = 100.0 * kPi / 180.0;
  parameters.angle_limits.fill(SteeringAngleLimits{-2.4, 2.4, 0.0, 0.05});
  return parameters;
}
}  // namespace

TEST(SwerveKinematicsTest, JerkLimitedSteeringRespectsLimitsAndArrivesAtRest)
{
  // ELECTRI-127: S-curve steering; acceleration ramps instead of stepping.
  const auto parameters = s_curve_parameters();
  SwerveSetpointGenerator generator{parameters};
  std::array<double, kSwerveModuleCount> angles{};
  generator.reset(angles);
  const double goal{45.0 * kPi / 180.0};
  const auto history = run_steering(generator, angles, goal, 4000);
  double previous_rate{0.0};
  double peak_rate{0.0};
  double peak_acceleration{0.0};
  for (std::size_t i = 1; i < history.size(); ++i) {
    const double rate{(history[i] - history[i - 1]) / 0.001};
    peak_rate = std::max(peak_rate, std::abs(rate));
    peak_acceleration = std::max(peak_acceleration, std::abs(rate - previous_rate) / 0.001);
    previous_rate = rate;
    EXPECT_LE(history[i], goal + 1e-12);  // never passes the goal
  }
  EXPECT_LE(peak_rate, parameters.maximum_steering_slew_radps * (1.0 + 1e-9));
  // Finite differences of a trapezoidal position integration: allow one jerk step of slack.
  EXPECT_LE(
    peak_acceleration, parameters.maximum_steering_acceleration_radps2 +
                         parameters.maximum_steering_jerk_radps3 * 0.001);
  EXPECT_NEAR(history.back(), goal, 1e-6);
  // Arrival is smooth: within 1e-5 rad of the goal the rate is already tiny (no step to 0).
  const auto near = std::find_if(
    history.begin() + 1, history.end(), [goal](double angle) { return goal - angle < 1e-5; });
  ASSERT_NE(near, history.end());
  EXPECT_LT((*near - *(near - 1)) / 0.001, 0.01);
}

TEST(SwerveKinematicsTest, JerkLimitedSteeringFollowsAGoalChangeWithoutOvershoot)
{
  SwerveSetpointGenerator generator{s_curve_parameters()};
  std::array<double, kSwerveModuleCount> angles{};
  generator.reset(angles);
  run_steering(generator, angles, 45.0 * kPi / 180.0, 1200);
  const auto history = run_steering(generator, angles, -30.0 * kPi / 180.0, 6000);
  for (const auto angle : history) {
    EXPECT_GE(angle, -30.0 * kPi / 180.0 - 1e-12);
  }
  EXPECT_NEAR(history.back(), -30.0 * kPi / 180.0, 1e-6);
}

TEST(SwerveKinematicsTest, JerkLimitNeedsAFiniteAccelerationLimit)
{
  SwerveSetpointParameters parameters{kPi, 0.1, 1.0};
  parameters.maximum_steering_jerk_radps3 = 1.0;  // acceleration still unlimited
  EXPECT_THROW(SwerveSetpointGenerator{parameters}, std::invalid_argument);
  parameters.maximum_steering_acceleration_radps2 = 1.0;
  parameters.maximum_steering_jerk_radps3 = 0.0;
  EXPECT_THROW(SwerveSetpointGenerator{parameters}, std::invalid_argument);
}
}  // namespace swerve_driver
