#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <type_traits>

#include "swerve_driver/relative_move_planner.hpp"

namespace swerve_driver
{
namespace
{
constexpr double kTolerance = 2e-11;

// Synthetic fixtures only; none of these values describe production hardware.
RelativeMoveConfig config()
{
  RelativeMoveConfig result;
  constexpr std::array<Translation2d, 4> locations{{{0.4, 0.3}, {0.4, -0.3}, {-0.4, 0.3}, {-0.4, -0.3}}};
  for (std::size_t i = 0; i < 4; ++i) {
    result.modules[i] = {locations[i], 0.08 + 0.01 * static_cast<double>(i),
      {-kPi, kPi, 0.05, 0.01}, 2.0 + static_cast<double>(i), 3.0 + static_cast<double>(i)};
  }
  result.max_translation_velocity_mps = 0.2;
  result.max_translation_acceleration_mps2 = 0.3;
  result.max_yaw_velocity_radps = 0.4;
  result.max_yaw_acceleration_radps2 = 0.5;
  return result;
}

RelativeMoveStart start()
{
  RelativeMoveStart result;
  for (std::size_t i = 0; i < 4; ++i) {
    result[i] = {0.02, 0.01, 0.5 * static_cast<double>(i)};
  }
  return result;
}

std::array<Translation2d, 4> locations(const RelativeMoveConfig & cfg)
{
  return {{cfg.modules[0].location, cfg.modules[1].location,
    cfg.modules[2].location, cfg.modules[3].location}};
}

// Independent integration of reconstructed wheel twist using long double arithmetic.
void integrate(Pose2d & pose, const ChassisDelta & delta)
{
  const long double theta = delta.dtheta_rad;
  const long double a = theta == 0.0L ? 1.0L : std::sin(theta) / theta;
  const long double b = theta == 0.0L ? 0.0L :
    2.0L * std::sin(theta / 2.0L) * std::sin(theta / 2.0L) / theta;
  const long double dx = a * delta.dx_m - b * delta.dy_m;
  const long double dy = b * delta.dx_m + a * delta.dy_m;
  pose.x_m += static_cast<double>(std::cos(pose.heading_rad) * dx - std::sin(pose.heading_rad) * dy);
  pose.y_m += static_cast<double>(std::sin(pose.heading_rad) * dx + std::cos(pose.heading_rad) * dy);
  pose.heading_rad += delta.dtheta_rad;
}

TEST(RelativeMovePlannerTest, WheelForwardIntegrationReachesSignedGoalsAndShortMoves)
{
  const std::array<Pose2d, 12> goals{{
    {0.7, 0.0, 0.0}, {-0.7, 0.0, 0.0}, {0.0, 0.7, 0.0}, {0.0, -0.7, 0.0},
    {0.0, 0.0, kPi / 12.0}, {0.0, 0.0, -kPi / 12.0},
    {0.4, -0.3, 0.25}, {-0.4, 0.3, -0.25}, {0.3, 0.2, 1e-8},
    {-0.3, -0.2, -1e-12}, {1e-8, -2e-8, 1e-8}, {1e-10, 0.0, 0.0}}};
  const auto cfg = config();
  for (const auto & goal : goals) {
    SCOPED_TRACE(::testing::Message() << goal.x_m << "," << goal.y_m << "," << goal.heading_rad);
    RelativeMovePlanner planner;
    auto initial = start();
    initial[0].drive_position_rad = 0.0;
    ASSERT_EQ(planner.configure(goal, cfg, initial), RelativeMoveError::none);
    const auto plan = planner.plan();
    std::array<SwerveModulePosition, 4> previous{};
    for (std::size_t i = 0; i < 4; ++i) {
      previous[i] = {initial[i].drive_position_rad * cfg.modules[i].wheel_radius_m,
        plan.modules[i].steering_target_rad, true};
    }
    Pose2d integrated;
    for (int step = 0; step < 500; ++step) {
      ASSERT_TRUE(planner.advance(plan.duration_s / 499.0));
      std::array<SwerveModulePosition, 4> current = previous;
      for (std::size_t i = 0; i < 4; ++i) {
        current[i].distance_m = planner.sample().drive_position_rad[i] * cfg.modules[i].wheel_radius_m;
      }
      const auto delta = wheel_chassis_delta_from_position_deltas(previous, current, locations(cfg));
      ASSERT_TRUE(delta.has_value());
      integrate(integrated, *delta);
      EXPECT_NEAR(integrated.x_m, planner.sample().planned_pose.x_m, kTolerance);
      EXPECT_NEAR(integrated.y_m, planner.sample().planned_pose.y_m, kTolerance);
      EXPECT_NEAR(integrated.heading_rad, planner.sample().planned_pose.heading_rad, kTolerance);
      previous = current;
    }
    EXPECT_EQ(planner.sample().state, RelativeMoveState::complete);
    EXPECT_DOUBLE_EQ(planner.sample().progress, 1.0);
    EXPECT_DOUBLE_EQ(planner.sample().progress_velocity, 0.0);
    EXPECT_NEAR(integrated.x_m, goal.x_m, kTolerance);
    EXPECT_NEAR(integrated.y_m, goal.y_m, kTolerance);
    EXPECT_NEAR(integrated.heading_rad, goal.heading_rad, kTolerance);
    EXPECT_NEAR(planner.sample().planned_pose.x_m, goal.x_m, 1e-15);
    EXPECT_NEAR(planner.sample().planned_pose.y_m, goal.y_m, 1e-15);
    EXPECT_DOUBLE_EQ(planner.sample().planned_pose.heading_rad, goal.heading_rad);
  }
}

TEST(RelativeMovePlannerTest, BodyYawAndPerWheelVelocityAndAccelerationLimits)
{
  for (int limiting_axis = 0; limiting_axis < 6; ++limiting_axis) {
    auto cfg = config();
    if (limiting_axis == 0) {cfg.max_translation_acceleration_mps2 = 0.001;}
    if (limiting_axis == 1) {cfg.max_yaw_velocity_radps = 0.003;}
    if (limiting_axis == 2) {cfg.max_yaw_acceleration_radps2 = 0.004;}
    if (limiting_axis == 3) {cfg.modules[2].max_wheel_velocity_radps = 0.04;}
    if (limiting_axis == 4) {cfg.modules[3].max_wheel_acceleration_radps2 = 0.005;}
    if (limiting_axis == 5) {cfg.max_translation_velocity_mps = 0.007;}
    RelativeMovePlanner planner;
    ASSERT_EQ(planner.configure({0.4, -0.3, 0.25}, cfg, start()), RelativeMoveError::none);
    const auto plan = planner.plan();
    for (int step = 0; step < 1001; ++step) {
      const auto sample = planner.sample();
      const double linear = std::hypot(sample.body_velocity.vx_mps, sample.body_velocity.vy_mps);
      EXPECT_LE(linear, cfg.max_translation_velocity_mps + 1e-14);
      EXPECT_LE(std::abs(sample.body_velocity.omega_radps), cfg.max_yaw_velocity_radps + 1e-14);
      const double body_acceleration = std::hypot(plan.body_log.dx_m, plan.body_log.dy_m) *
        std::hypot(sample.progress_acceleration,
        plan.body_log.dtheta_rad * sample.progress_velocity * sample.progress_velocity);
      EXPECT_LE(body_acceleration, cfg.max_translation_acceleration_mps2 + 1e-14);
      EXPECT_LE(std::abs(plan.body_log.dtheta_rad * sample.progress_acceleration),
        cfg.max_yaw_acceleration_radps2 + 1e-14);
      for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_LE(std::abs(sample.drive_velocity_radps[i]), cfg.modules[i].max_wheel_velocity_radps + 1e-14);
        EXPECT_LE(std::abs(plan.modules[i].drive_travel_rad * sample.progress_acceleration),
          cfg.modules[i].max_wheel_acceleration_radps2 + 1e-14);
      }
      ASSERT_TRUE(planner.advance(plan.duration_s / 1000.0));
    }
  }
}

TEST(RelativeMovePlannerTest, TriangularShortMoveAndTrapezoidalLongMove)
{
  RelativeMovePlanner short_move;
  ASSERT_EQ(short_move.configure({0.0001, 0.0, 0.0}, config(), start()), RelativeMoveError::none);
  EXPECT_NEAR(short_move.plan().cruise_duration_s, 0.0, 1e-15);
  EXPECT_LT(short_move.plan().peak_progress_velocity, short_move.plan().progress_velocity_limit);
  RelativeMovePlanner long_move;
  ASSERT_EQ(long_move.configure({0.7, 0.0, 0.0}, config(), start()), RelativeMoveError::none);
  EXPECT_GT(long_move.plan().cruise_duration_s, 0.0);
  EXPECT_DOUBLE_EQ(long_move.plan().peak_progress_velocity, long_move.plan().progress_velocity_limit);
}

TEST(RelativeMovePlannerTest, AnalyticSamplingIsIndependentOfTimePartition)
{
  RelativeMovePlanner whole, partitioned;
  ASSERT_EQ(whole.configure({0.4, 0.1, -0.2}, config(), start()), RelativeMoveError::none);
  ASSERT_EQ(partitioned.configure({0.4, 0.1, -0.2}, config(), start()), RelativeMoveError::none);
  const double dt = whole.plan().duration_s * 0.83;
  ASSERT_TRUE(whole.advance(dt));
  for (double fraction : {0.02, 0.3, 0.001, 0.6, 0.079}) {
    ASSERT_TRUE(partitioned.advance(dt * fraction));
  }
  EXPECT_NEAR(whole.sample().progress, partitioned.sample().progress, 1e-14);
  EXPECT_NEAR(whole.sample().progress_velocity, partitioned.sample().progress_velocity, 1e-14);
  ASSERT_TRUE(whole.advance(std::numeric_limits<double>::max()));
  EXPECT_EQ(whole.sample().state, RelativeMoveState::complete);
  auto terminal = whole.sample();
  ASSERT_TRUE(whole.advance(0.25));
  EXPECT_EQ(whole.sample().drive_position_rad, terminal.drive_position_rad);
  whole.cancel();
  EXPECT_EQ(whole.sample().state, RelativeMoveState::complete);
}

TEST(RelativeMovePlannerTest, CancelInEachPhaseDeceleratesOnSamePathThenHolds)
{
  for (double fraction : {0.0, 0.01, 0.2, 0.5, 0.95, 0.999999}) {
    RelativeMovePlanner planner;
    ASSERT_EQ(planner.configure({-0.4, 0.2, -0.2}, config(), start()), RelativeMoveError::none);
    if (fraction > 0.0) {ASSERT_TRUE(planner.advance(planner.plan().duration_s * fraction));}
    const auto before = planner.sample();
    const auto plan = planner.plan();
    const double stop_time = before.progress_velocity / plan.progress_acceleration;
    const double expected_stop = before.progress + 0.5 * before.progress_velocity * stop_time;
    planner.cancel();
    EXPECT_EQ(planner.sample().drive_position_rad, before.drive_position_rad);
    EXPECT_EQ(planner.sample().drive_velocity_radps, before.drive_velocity_radps);
    EXPECT_DOUBLE_EQ(planner.sample().progress, before.progress);
    double elapsed = 0.0;
    double last_velocity = before.progress_velocity;
    double last_progress = before.progress;
    for (double portion : {0.013, 0.32, 0.0001, 0.49, 0.2, 1.0}) {
      const double dt = stop_time > 0.0 ? stop_time * portion : 0.01;
      ASSERT_TRUE(planner.advance(dt));
      elapsed = std::min(stop_time, elapsed + dt);
      const auto sample = planner.sample();
      EXPECT_NEAR(sample.progress, before.progress + before.progress_velocity * elapsed -
        0.5 * plan.progress_acceleration * elapsed * elapsed, 1e-14);
      EXPECT_LE(sample.progress_velocity, last_velocity);
      EXPECT_GE(sample.progress, last_progress - 1e-15);
      EXPECT_LE(sample.progress, 1.0 + 1e-15);
      for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(planner.plan().modules[i].steering_target_rad, plan.modules[i].steering_target_rad);
        EXPECT_NEAR(sample.drive_position_rad[i], plan.modules[i].drive_start_rad +
          plan.modules[i].drive_travel_rad * sample.progress, 1e-14);
      }
      planner.cancel();  // repeated requests must not restart deceleration
      last_velocity = sample.progress_velocity;
      last_progress = sample.progress;
    }
    EXPECT_EQ(planner.sample().state, RelativeMoveState::canceled);
    EXPECT_NEAR(planner.sample().progress, expected_stop, 1e-14);
    EXPECT_DOUBLE_EQ(planner.sample().progress_velocity, 0.0);
    const auto stopped = planner.sample();
    ASSERT_TRUE(planner.advance(10.0));
    EXPECT_EQ(planner.sample().drive_position_rad, stopped.drive_position_rad);
  }
}

TEST(RelativeMovePlannerTest, BranchesStayWithinMechanicalEndpointsAndKeepFeedbackSeparate)
{
  auto cfg = config();
  auto initial = start();
  initial[0] = {kPi - 0.1, kPi - 0.11, 3.0};
  initial[1] = {-kPi + 0.1, -kPi + 0.11, -2.0};
  const double desired = -179.0 * kPi / 180.0;
  RelativeMovePlanner planner;
  ASSERT_EQ(planner.configure({0.1 * std::cos(desired), 0.1 * std::sin(desired), 0.0}, cfg, initial),
    RelativeMoveError::none);
  for (std::size_t i = 0; i < 4; ++i) {
    const auto & module = planner.plan().modules[i];
    EXPECT_TRUE(steering_angle_within_limits(module.steering_target_rad, cfg.modules[i].steering_limits));
    EXPECT_LT(module.drive_travel_rad, 0.0);
    EXPECT_NEAR(module.steering_target_rad, kPi / 180.0, 1e-14);
    EXPECT_DOUBLE_EQ(module.steering_start_rad, initial[i].steering_motor_rad);
    EXPECT_NEAR(module.steering_alignment_error_rad,
      module.steering_target_rad - initial[i].steering_measured_rad, 1e-14);
  }
  EXPECT_GT(std::abs(planner.plan().modules[0].steering_target_rad - initial[0].steering_motor_rad), 3.0);
  EXPECT_EQ(planner.sample().drive_position_rad[0], initial[0].drive_position_rad);
}

TEST(RelativeMovePlannerTest, MixedBranchesAndShiftedMechanicalIntervalsAreFeasible)
{
  auto cfg = config();
  auto initial = start();
  for (std::size_t i = 0; i < 4; ++i) {
    cfg.modules[i].steering_limits = {0.0, 2.0 * kPi, 0.0, 0.0};
    initial[i].steering_motor_rad = i % 2 == 0 ? 0.0 : kPi;
    initial[i].steering_measured_rad = initial[i].steering_motor_rad;
  }
  RelativeMovePlanner planner;
  ASSERT_EQ(planner.configure({0.1, 0.0, 0.0}, cfg, initial), RelativeMoveError::none);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(planner.plan().modules[i].steering_target_rad, initial[i].steering_motor_rad);
    EXPECT_EQ(planner.plan().modules[i].drive_travel_rad > 0.0, i % 2 == 0);
  }
  // The shared mechanical contract requires a safe span of at least pi.
  cfg.modules[0].steering_limits = {-0.1, 0.1, 0.0, 0.0};
  RelativeMovePlanner infeasible;
  EXPECT_EQ(infeasible.configure({0.0, 0.1, 0.0}, cfg, initial), RelativeMoveError::invalid_config);
}

TEST(RelativeMovePlannerTest, ModuleAtInstantaneousCenterHoldsItsMotorTarget)
{
  auto cfg = config();
  cfg.modules[0].location = {0.0, 0.0};
  RelativeMovePlanner planner;
  ASSERT_EQ(planner.configure({0.0, 0.0, 0.1}, cfg, start()), RelativeMoveError::none);
  EXPECT_DOUBLE_EQ(planner.plan().modules[0].drive_travel_rad, 0.0);
  EXPECT_DOUBLE_EQ(planner.plan().modules[0].steering_target_rad, start()[0].steering_motor_rad);
  ASSERT_TRUE(planner.advance(planner.plan().duration_s));
  EXPECT_DOUBLE_EQ(planner.sample().drive_position_rad[0], start()[0].drive_position_rad);
}

TEST(RelativeMovePlannerTest, ZeroGoalCompletesWithoutSteeringCorrection)
{
  RelativeMovePlanner planner;
  ASSERT_EQ(planner.configure({}, config(), start()), RelativeMoveError::none);
  EXPECT_EQ(planner.sample().state, RelativeMoveState::complete);
  EXPECT_DOUBLE_EQ(planner.plan().duration_s, 0.0);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(planner.plan().modules[i].steering_target_rad, start()[i].steering_motor_rad);
    EXPECT_DOUBLE_EQ(planner.sample().drive_position_rad[i], start()[i].drive_position_rad);
  }
}

TEST(RelativeMovePlannerTest, AdmissionRangeFiniteAndMissingConfigurationChecks)
{
  RelativeMovePlanner planner;
  EXPECT_EQ(planner.configure({0.700001, 0.0, 0.0}, config(), start()), RelativeMoveError::goal_out_of_range);
  EXPECT_EQ(planner.configure({0.5, 0.5, 0.0}, config(), start()), RelativeMoveError::goal_out_of_range);
  EXPECT_EQ(planner.configure({0.0, 0.0, -kPi / 12.0 - 1e-12}, config(), start()), RelativeMoveError::goal_out_of_range);
  for (double invalid : {std::numeric_limits<double>::infinity(), kRelativeMoveRequired}) {
    EXPECT_EQ(planner.configure({invalid, 0.0, 0.0}, config(), start()), RelativeMoveError::invalid_goal);
    EXPECT_EQ(planner.configure({0.0, invalid, 0.0}, config(), start()), RelativeMoveError::invalid_goal);
    EXPECT_EQ(planner.configure({0.0, 0.0, invalid}, config(), start()), RelativeMoveError::invalid_goal);
  }
  EXPECT_EQ(planner.configure({}, RelativeMoveConfig{}, start()), RelativeMoveError::invalid_config);
  EXPECT_EQ(planner.configure({}, config(), RelativeMoveStart{}), RelativeMoveError::invalid_start);
  for (int field = 0; field < 12; ++field) {
    auto cfg = config();
    if (field == 0) {cfg.max_translation_velocity_mps = 0.200001;}
    if (field == 1) {cfg.max_translation_acceleration_mps2 = 0.0;}
    if (field == 2) {cfg.max_yaw_velocity_radps = kRelativeMoveRequired;}
    if (field == 3) {cfg.max_yaw_acceleration_radps2 = -1.0;}
    if (field == 4) {cfg.modules[0].wheel_radius_m = 0.0;}
    if (field == 5) {cfg.modules[0].max_wheel_velocity_radps = -1.0;}
    if (field == 6) {cfg.modules[0].max_wheel_acceleration_radps2 = kRelativeMoveRequired;}
    if (field == 7) {cfg.modules[0].location.x = kRelativeMoveRequired;}
    if (field == 8) {cfg.modules[0].location = cfg.modules[1].location;}
    if (field == 9) {cfg.modules[0].steering_limits.margin_rad = -1.0;}
    if (field == 10) {cfg.modules[0].steering_limits.measurement_tolerance_rad = kRelativeMoveRequired;}
    if (field == 11) {cfg.modules[0].steering_limits.maximum_rad = 3.0 * kPi;}
    EXPECT_EQ(planner.configure({}, cfg, start()), RelativeMoveError::invalid_config) << field;
  }
  auto initial = start();
  initial[0].steering_motor_rad = kPi;
  EXPECT_EQ(planner.configure({}, config(), initial), RelativeMoveError::invalid_start);
  initial = start();
  initial[1].steering_measured_rad = kPi + 0.02;
  EXPECT_EQ(planner.configure({}, config(), initial), RelativeMoveError::invalid_start);
}

TEST(RelativeMovePlannerTest, ExtremeArithmeticFailsWithoutPublishingInvalidPlan)
{
  RelativeMovePlanner planner;
  auto cfg = config();
  cfg.modules[0].wheel_radius_m = std::numeric_limits<double>::denorm_min();
  EXPECT_EQ(planner.configure({0.7, 0.0, 0.0}, cfg, start()), RelativeMoveError::numeric_range);
  auto initial = start();
  initial[0].drive_position_rad = std::numeric_limits<double>::max();
  EXPECT_EQ(planner.configure({0.1, 0.0, 0.0}, config(), initial), RelativeMoveError::numeric_range);
  EXPECT_EQ(planner.sample().state, RelativeMoveState::unconfigured);
}

TEST(RelativeMovePlannerTest, InvalidDtAndActiveReplacementNeverResetTargets)
{
  RelativeMovePlanner planner;
  EXPECT_FALSE(planner.advance(0.1));
  planner.cancel();
  EXPECT_EQ(planner.sample().state, RelativeMoveState::unconfigured);
  ASSERT_EQ(planner.configure({0.4, 0.1, 0.1}, config(), start()), RelativeMoveError::none);
  ASSERT_TRUE(planner.advance(0.1));
  const auto before = planner.sample();
  for (double dt : {0.0, -0.1, kRelativeMoveRequired, std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(planner.advance(dt));
    EXPECT_EQ(planner.sample().drive_position_rad, before.drive_position_rad);
    EXPECT_DOUBLE_EQ(planner.sample().progress_velocity, before.progress_velocity);
  }
  EXPECT_EQ(planner.configure({}, config(), start()), RelativeMoveError::busy);
  EXPECT_EQ(planner.sample().drive_position_rad, before.drive_position_rad);
  planner.cancel();
  EXPECT_EQ(planner.configure({}, config(), start()), RelativeMoveError::busy);
  ASSERT_TRUE(planner.advance(100.0));
  const auto terminal = planner.sample();
  EXPECT_EQ(planner.configure({}, RelativeMoveConfig{}, start()), RelativeMoveError::invalid_config);
  EXPECT_EQ(planner.sample().drive_position_rad, terminal.drive_position_rad);
  EXPECT_EQ(planner.configure({}, config(), start()), RelativeMoveError::none);
}

static_assert(std::is_trivially_copyable<RelativeMovePlanner>::value,
  "Planner must remain fixed-size and independently copyable for offline evaluation");
}  // namespace
}  // namespace swerve_driver
