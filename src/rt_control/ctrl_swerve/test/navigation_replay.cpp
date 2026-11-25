#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include "ctrl_swerve/control_core.hpp"
#include "navigation_replay.hpp"

namespace navigation_replay
{
namespace
{
using ctrl_swerve::ChassisSpeeds;
using ctrl_swerve::CoreConfig;
using ctrl_swerve::kPi;

CoreConfig replay_config(bool align_first = false)
{
  // Deliberately synthetic geometry, not the bundle's hardware mapping/calibration.
  CoreConfig c;
  c.locations = {{{0.5, 0.4}, {0.5, -0.4}, {-0.5, 0.4}, {-0.5, -0.4}}};
  c.wheel_radius.fill(0.1);
  c.steering_min.fill(-3.0 * kPi / 4.0);
  c.steering_max.fill(3.0 * kPi / 4.0);
  c.steering_limit_margin.fill(kPi / 12.0);
  c.steering_limit_tolerance.fill(0.0017453292519943296);
  c.max_linear_speed = 1.0;
  c.max_angular_speed = 0.8;
  c.max_wheel_speed = 2.0;
  c.max_wheel_acceleration = 2.0;
  c.velocity_deadband = 0.001;
  c.max_encoder_difference = kPi / 180.0;
  c.max_update_period = 0.02;
  c.slip_residual_threshold = 1.0;
  c.max_linear_acceleration = 1.0;
  c.max_angular_acceleration = kPi / 8.0;
  c.max_body_jerk = 4.0;
  c.setpoint.alignment_threshold_rad = align_first ? kPi / 36.0 : kPi / 2.0;
  c.setpoint.alignment_release_rad = align_first ? kPi / 60.0 : kPi / 2.0;
  c.setpoint.flip_hysteresis_rad = kPi / 60.0;
  c.setpoint.maximum_steering_slew_radps = kPi / 2.0;
  c.setpoint.maximum_steering_acceleration_radps2 = kPi;
  c.setpoint.maximum_steering_jerk_radps3 = 1.7453292519943295;
  c.setpoint.steering_angle_deadband_rad = 0.0017453292519943296;
  return c;
}

Config bridge_config(const CoreConfig & c)
{
  Config out;
  for (std::size_t i = 0; i < 4; ++i) {
    out.x[i] = c.locations[i].x;
    out.y[i] = c.locations[i].y;
  }
  out.radius = c.wheel_radius;
  out.minimum = c.steering_min;
  out.maximum = c.steering_max;
  out.margin = c.steering_limit_margin;
  out.tolerance = c.steering_limit_tolerance;
  out.linear_speed = c.max_linear_speed;
  out.angular_speed = c.max_angular_speed;
  out.rotation_speed = c.max_rotation_speed;
  out.wheel_speed = c.max_wheel_speed;
  out.wheel_acceleration = c.max_wheel_acceleration;
  out.linear_acceleration = c.max_linear_acceleration;
  out.angular_acceleration = c.max_angular_acceleration;
  out.rotation_acceleration = c.max_rotation_acceleration;
  out.body_jerk = c.max_body_jerk;
  out.deadband = c.velocity_deadband;
  out.encoder_difference = c.max_encoder_difference;
  out.update_period = c.max_update_period;
  out.slip_threshold = c.slip_residual_threshold;
  out.encoder_window = c.encoder_fault_window;
  out.gate = c.setpoint.alignment_threshold_rad;
  out.release = c.setpoint.alignment_release_rad;
  out.hysteresis = c.setpoint.flip_hysteresis_rad;
  out.steering_speed = c.setpoint.maximum_steering_slew_radps;
  out.steering_acceleration = c.setpoint.maximum_steering_acceleration_radps2;
  out.steering_jerk = c.setpoint.maximum_steering_jerk_radps3;
  out.heading_epsilon = c.setpoint.translation_heading_epsilon_rad;
  out.angle_deadband = c.setpoint.steering_angle_deadband_rad;
  return out;
}

State exported_state(
  const ctrl_swerve::ExecutionSnapshot & snapshot,
  const ctrl_swerve::ModuleFeedbackArray & measurements, const CoreConfig & c)
{
  State out;
  out.velocity = {
    snapshot.body_reference.vx_mps, snapshot.body_reference.vy_mps,
    snapshot.body_reference.omega_radps};
  out.acceleration = {
    snapshot.body_acceleration.vx_mps, snapshot.body_acceleration.vy_mps,
    snapshot.body_acceleration.omega_radps};
  out.gated = snapshot.gated;
  out.waiting = snapshot.waiting;
  out.branch = snapshot.translation_branch;
  out.status = static_cast<int>(snapshot.status);
  for (std::size_t i = 0; i < 4; ++i) {
    const auto & m = snapshot.modules[i];
    const auto & actual = measurements[i];
    out.modules[i] = {
      actual.steering_position,
      actual.steering_angle,
      actual.wheel_velocity * c.wheel_radius[i],
      actual.wheel_position,
      m.steering_reference,
      m.steering_rate,
      m.steering_acceleration,
      m.wheel_reference,
      m.steering_target,
      m.steering_error,
      m.reversed,
      m.holding};
  }
  return out;
}

void expect_same(const State & driver, const State & reference)
{
  constexpr double tolerance = 1e-5;
  EXPECT_EQ(driver.status, reference.status);
  EXPECT_EQ(driver.gated, reference.gated);
  EXPECT_EQ(driver.waiting, reference.waiting);
  EXPECT_EQ(driver.branch, reference.branch);
  for (std::size_t i = 0; i < 3; ++i) {
    SCOPED_TRACE("body axis=" + std::to_string(i));
    EXPECT_NEAR(driver.velocity[i], reference.velocity[i], tolerance);
    EXPECT_NEAR(driver.acceleration[i], reference.acceleration[i], tolerance);
  }
  for (std::size_t i = 0; i < 4; ++i) {
    SCOPED_TRACE("module=" + std::to_string(i));
    const auto & d = driver.modules[i];
    const auto & r = reference.modules[i];
    EXPECT_EQ(d.reversed, r.reversed);
    EXPECT_EQ(d.holding, r.holding);
    EXPECT_NEAR(d.goal, r.goal, tolerance);
    EXPECT_NEAR(d.error, r.error, tolerance);
    EXPECT_NEAR(d.reference, r.reference, tolerance);
    EXPECT_NEAR(d.rate, r.rate, tolerance);
    EXPECT_NEAR(d.acceleration, r.acceleration, tolerance);
    EXPECT_NEAR(d.wheel_reference, r.wheel_reference, tolerance);
  }
}

class Replay
{
public:
  explicit Replay(
    CoreConfig initial_config = replay_config(), bool spin_aligned = false,
    double initial_angle = 0.0)
  : config(initial_config), bridge(bridge_config(initial_config)), core(initial_config)
  {
    for (std::size_t i = 0; i < 4; ++i) {
      auto & m = feedback[i];
      m.valid = m.enabled = m.encoder_valid = true;
      m.steering_position = m.steering_angle = initial_angle;
      if (spin_aligned) {
        const auto desired = ctrl_swerve::SwerveModuleState{
          1.0, std::atan2(config.locations[i].x, -config.locations[i].y)};
        const auto optimized = ctrl_swerve::optimize_module(
          desired, 0.0, false, config.setpoint.flip_hysteresis_rad,
          {config.steering_min[i], config.steering_max[i], config.steering_limit_margin[i],
           config.steering_limit_tolerance[i]});
        m.steering_position = m.steering_angle = optimized.target_angle_rad;
      }
    }
    if (!core.activate(feedback)) {
      throw std::runtime_error("driver activation rejected replay feedback");
    }
  }

  State seed() const { return exported_state(core.execution_snapshot(), feedback, config); }

  State tick(ChassisSpeeds command, bool requested = false, double dt = 0.001, bool fresh = true)
  {
    const auto before = seed();
    const auto output = core.update(feedback, command, fresh, dt, std::nullopt, requested);
    const auto after = exported_state(core.execution_snapshot(), feedback, config);
    last_reference = reference_step(
      bridge, before, {command.vx_mps, command.vy_mps, command.omega_radps}, dt,
      core.execution_snapshot().rotation_active, fresh);
    SCOPED_TRACE("cycle=" + std::to_string(cycles));
    expect_same(after, last_reference);
    // Identical ideal measurements feed both implementations next time. They do
    // not replace either implementation's internal reference/holding/branch state.
    for (std::size_t i = 0; i < 4; ++i) {
      feedback[i].steering_position = feedback[i].steering_angle = output.steering_position[i];
      feedback[i].wheel_velocity = output.drive_velocity[i];
      feedback[i].wheel_position += output.drive_velocity[i] * dt;
    }
    ++cycles;
    return after;
  }

  void run(ChassisSpeeds command, int count, bool requested = false, double dt = 0.001)
  {
    for (int i = 0; i < count; ++i) {
      tick(command, requested, dt);
      if (testing::Test::HasFailure()) {
        return;
      }
    }
  }

  CoreConfig config;
  Config bridge;
  ctrl_swerve::ControlCore core;
  ctrl_swerve::ModuleFeedbackArray feedback{};
  State last_reference;
  int cycles{0};
};

TEST(NavigationReplay, StoppedAdmissionAuthorizedRotationAndSteadyRevoke)
{
  Replay replay{replay_config(), true};
  replay.run({}, 60, true);
  ASSERT_FALSE(HasFailure());
  ASSERT_TRUE(replay.core.execution_snapshot().rotation_active);
  replay.run({0.0, 0.0, 2.0}, 4000, true);
  ASSERT_FALSE(HasFailure());
  EXPECT_NEAR(replay.seed().velocity[2], 1.5, 1e-9);
  replay.run({0.0, 0.0, 2.0}, 4000, false);
  ASSERT_FALSE(HasFailure());
  EXPECT_FALSE(replay.core.execution_snapshot().rotation_active);
  EXPECT_NEAR(replay.seed().velocity[2], 0.8, 1e-9);
  replay.run({}, 4000);
  ASSERT_FALSE(HasFailure());
}

TEST(NavigationReplay, HighRateRotationThenTranslationWaitsForOrdinaryEnvelope)
{
  Replay replay{replay_config(), true};
  replay.run({}, 60, true);
  replay.run({0.0, 0.0, 1.5}, 4000, true);
  ASSERT_FALSE(HasFailure());
  for (int i = 0; i < 100; ++i) {
    const auto state = replay.tick({0.4, 0.0, 0.8}, true);
    ASSERT_FALSE(HasFailure());
    EXPECT_DOUBLE_EQ(state.velocity[0], 0.0);
    EXPECT_DOUBLE_EQ(state.velocity[1], 0.0);
    EXPECT_FALSE(replay.core.execution_snapshot().rotation_active);
  }
  replay.run({0.4, 0.0, 0.8}, 6000);
  ASSERT_FALSE(HasFailure());
}

TEST(NavigationReplay, PlanarAxesNegativeCommandsDiagonalRetargetAndLimits)
{
  Replay replay;
  replay.run({}, 20);
  replay.run({2.0, 0.0, 0.0}, 2000);
  ASSERT_FALSE(HasFailure());
  EXPECT_NEAR(replay.seed().velocity[0], 1.0, 1e-9);
  replay.run({-0.7, 0.0, 0.0}, 3500);
  ASSERT_FALSE(HasFailure());
  EXPECT_EQ(replay.seed().branch, 1);
  replay.run({}, 3500);
  replay.run({0.0, -0.7, 0.0}, 5000);
  ASSERT_FALSE(HasFailure());
  replay.run({0.3, 0.5, -0.3}, 1200);
  replay.run({2.0, 2.0, 2.0}, 3500);
  ASSERT_FALSE(HasFailure());
  const auto state = replay.seed();
  EXPECT_LE(std::hypot(state.velocity[0], state.velocity[1]), 1.0 + 1e-9);
  EXPECT_LE(std::abs(state.velocity[2]), 0.8 + 1e-9);
  replay.run({}, 4000);
  ASSERT_FALSE(HasFailure());
}

TEST(NavigationReplay, AlignmentGateHoldingNearTravelLimitAndVariableDt)
{
  Replay replay{replay_config(true), false, 2.08};
  bool saw_gate = false;
  for (int i = 0; i < 3500; ++i) {
    const double dt = i % 2 == 0 ? 0.001 : 0.003;
    const auto state = replay.tick({-0.4, 0.2, 0.0}, false, dt);
    ASSERT_FALSE(HasFailure());
    saw_gate = saw_gate || state.gated;
  }
  EXPECT_TRUE(saw_gate);
  replay.run({}, 4000);
  ASSERT_FALSE(HasFailure());
  for (const auto & m : replay.seed().modules) {
    EXPECT_TRUE(m.holding);
  }
  // Timeout exercises the same ramp-to-hold update, rather than inferring hold from zero speed.
  replay.tick({}, false, 0.001, false);
  ASSERT_FALSE(HasFailure());
}

TEST(NavigationReplay, MidRampRevokeMatchesReferenceTransientBoundsAndConvergence)
{
  for (const bool translate : {false, true}) {
    SCOPED_TRACE(translate ? "revoke into translation" : "revoke into ordinary rotation");
    Replay replay{replay_config(), true};
    replay.run({}, 60, true);
    replay.run({0.0, 0.0, 1.5}, 900, true);
    ASSERT_FALSE(HasFailure());
    auto previous = replay.seed();
    ASSERT_GT(previous.velocity[2], 0.8);
    ASSERT_GT(previous.acceleration[2], 0.0);
    const ChassisSpeeds command{translate ? 0.4 : 0.0, 0.0, 1.5};
    bool decelerated = false;
    for (int i = 0; i < 6000; ++i) {
      const auto after = replay.tick(command, false);
      ASSERT_FALSE(HasFailure());
      EXPECT_FALSE(replay.core.execution_snapshot().rotation_active);
      EXPECT_LE(std::abs(after.acceleration[2]), kPi / 8.0 + 1e-9);
      EXPECT_LE(std::abs(after.velocity[2] - previous.velocity[2]), kPi / 8.0 * 0.001 + 1e-9);
      if (i == 0) {
        EXPECT_GT(after.velocity[2], previous.velocity[2]);
        std::cout << std::setprecision(17)
                  << "REFERENCE_REVOKE dt=0.001 before_wz=" << previous.velocity[2]
                  << " before_alpha=" << previous.acceleration[2]
                  << " driver_wz=" << after.velocity[2] << " driver_alpha=" << after.acceleration[2]
                  << " reference_wz=" << replay.last_reference.velocity[2]
                  << " reference_alpha=" << replay.last_reference.acceleration[2] << '\n';
      }
      if (
        std::hypot(previous.velocity[0], previous.velocity[1]) <= replay.config.velocity_deadband &&
        (std::abs(previous.velocity[2]) > replay.config.max_angular_speed ||
         std::abs(previous.acceleration[2]) > replay.config.max_angular_acceleration)) {
        EXPECT_DOUBLE_EQ(after.velocity[0], 0.0);
        EXPECT_DOUBLE_EQ(after.velocity[1], 0.0);
      }
      decelerated = decelerated || after.velocity[2] < previous.velocity[2];
      previous = after;
    }
    EXPECT_TRUE(decelerated);
    EXPECT_NEAR(previous.velocity[2], 0.8, 1e-9);
    EXPECT_NEAR(previous.velocity[0], command.vx_mps, 1e-9);
    EXPECT_NEAR(previous.velocity[1], 0.0, 1e-9);
  }
}
}  // namespace
}  // namespace navigation_replay
