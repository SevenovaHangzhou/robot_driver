#include <gtest/gtest.h>
#include <limits>
#include "relative_move_fixture.hpp"
using namespace swerve_driver;

TEST(RelativeMoveSession, RequiredConfigurationAndStartup)
{
  RelativeMoveSession s;
  EXPECT_FALSE(s.configure({}));
  EXPECT_EQ(s.state().mode, ChassisMode::unknown);
  EXPECT_FALSE(s.state().ready);
  auto cfg = session_config(); cfg.motion.modules[0].wheel_radius_m = 0;
  EXPECT_FALSE(s.configure(cfg));
  EXPECT_TRUE(s.configure(session_config()));
  EXPECT_FALSE(s.configure(session_config()));
}
TEST(RelativeMoveSession, ModeRequiresConfirmationStationaryAndReadback)
{
  SessionRig r;
  EXPECT_FALSE(r.session.start(request()));
  EXPECT_EQ(r.session.set_mode(ChassisMode::operation, false), 1);
  r.feedback.steering_velocity[0] = 0.1; r.session.update(0.01, r.feedback);
  EXPECT_EQ(r.session.set_mode(ChassisMode::operation, true), 2);
  r.feedback.steering_velocity.fill(0);
  for (int i = 0; i < 5; ++i) {r.session.update(0.01, r.feedback);}
  EXPECT_EQ(r.session.set_mode(ChassisMode::operation, true), 0);
  EXPECT_EQ(r.session.state().mode, ChassisMode::navigation);
  EXPECT_FALSE(r.session.state().ready);
  EXPECT_EQ(r.session.set_mode(ChassisMode::navigation, true), 3);
  r.feedback.drive_mode[0] = 8; r.session.update(0.01, r.feedback);
  EXPECT_EQ(r.session.state().mode, ChassisMode::unknown);
  for (int i = 0; i < 110; ++i) {r.session.update(0.01, r.feedback);}
  EXPECT_EQ(r.session.state().fault, MoveFault::mode_switch);
  EXPECT_FALSE(r.session.state().ready);
}
TEST(RelativeMoveSession, AdmissionRejectsInvalidImuEveryGoalAndBoundsAndBusy)
{
  SessionRig r; r.operation();
  r.feedback.imu_age = 1; r.session.update(0.01, r.feedback);
  EXPECT_FALSE(r.session.start(request({})));
  EXPECT_FALSE(r.session.start(request()));
  r.feedback.imu_age = 0; r.session.update(0.01, r.feedback);
  EXPECT_FALSE(r.session.start(request({0.701, 0, 0})));
  EXPECT_FALSE(r.session.start(request({0, 0, kPi / 12 + 0.001})));
  auto req = request(); req.limits.max_translation_velocity_mps = 0.201;
  EXPECT_FALSE(r.session.start(req));
  req = request(); req.max_duration = 0.5; EXPECT_FALSE(r.session.start(req));
  req = request(); req.limits.max_yaw_acceleration_radps2 = 0; EXPECT_FALSE(r.session.start(req));
  ASSERT_TRUE(r.session.start(request()));
  EXPECT_FALSE(r.session.start(request()));
  EXPECT_EQ(r.session.set_mode(ChassisMode::navigation, true), 3);
  EXPECT_EQ(r.session.reset_fault(true), 3);
}
TEST(RelativeMoveSession, MixedMoveUsesMeasuredEstimateAndSettles)
{
  SessionRig r; r.operation(); const auto goal = Pose2d{0.3, -0.2, 0.15};
  ASSERT_TRUE(r.session.start(request(goal)));
  r.finish();
  ASSERT_FALSE(r.session.state().active);
  EXPECT_EQ(r.session.state().result, MoveResult::succeeded);
  EXPECT_NEAR(r.session.state().actual.x_m, goal.x_m, 1e-8);
  EXPECT_NEAR(r.session.state().actual.y_m, goal.y_m, 1e-8);
  EXPECT_NEAR(r.session.state().actual.heading_rad, goal.heading_rad, 1e-8);
  EXPECT_EQ(r.session.state().quality, 0U);
  const auto hold = r.session.output();
  for (int i = 0; i < 10; ++i) {r.tick();}
  EXPECT_EQ(hold.steering_position, r.session.output().steering_position);
  EXPECT_EQ(hold.wheel_position, r.session.output().wheel_position);
}
TEST(RelativeMoveSession, ZeroGoalKeepsTargets)
{
  SessionRig r; r.operation(); const auto hold = r.session.output();
  ASSERT_TRUE(r.session.start(request({})));
  EXPECT_EQ(r.session.state().phase, MovePhase::holding);
  r.finish();
  EXPECT_EQ(r.session.state().result, MoveResult::succeeded);
  EXPECT_EQ(r.session.state().progress, 1);
  EXPECT_EQ(hold.steering_position, r.session.output().steering_position);
  EXPECT_EQ(hold.wheel_position, r.session.output().wheel_position);
}
TEST(RelativeMoveSession, CancelAlignmentDeceleratesSteeringWithoutWheelTravel)
{
  SessionRig r; r.operation(); ASSERT_TRUE(r.session.start(request({0, 0.5, 0})));
  for (int i = 0; i < 20; ++i) {r.tick();}
  const auto before = r.session.output();
  r.session.cancel(); r.session.cancel();
  EXPECT_EQ(before.steering_position, r.session.output().steering_position);
  EXPECT_EQ(before.steering_velocity, r.session.output().steering_velocity);
  r.finish();
  EXPECT_EQ(r.session.state().result, MoveResult::canceled);
  EXPECT_EQ(r.session.output().wheel_position, before.wheel_position);
  EXPECT_GT(r.session.output().steering_position[0], before.steering_position[0]);
  EXPECT_LT(r.session.output().steering_position[0], r.session.plan().modules[0].steering_target_rad);
}
TEST(RelativeMoveSession, CancelAccelerationCruiseAndDecelerationKeepsPath)
{
  for (double progress : {0.001, 0.5, 0.99}) {
    SessionRig r; r.operation(); ASSERT_TRUE(r.session.start(request({0.5, 0.1, 0.1})));
    for (int i = 0; i < 2000 && r.session.state().progress < progress; ++i) {r.tick();}
    const auto before = r.session.output();
    r.session.cancel(); EXPECT_EQ(before.wheel_position, r.session.output().wheel_position);
    EXPECT_EQ(before.wheel_velocity, r.session.output().wheel_velocity);
    r.finish(); EXPECT_EQ(r.session.state().result, MoveResult::canceled);
    EXPECT_LE(r.session.state().progress, 1);
    for (std::size_t i = 0; i < 4; ++i) {
      EXPECT_NEAR(r.session.output().wheel_position[i], r.session.plan().modules[i].drive_start_rad +
        r.session.plan().modules[i].drive_travel_rad * r.session.state().progress, 1e-10);
      EXPECT_EQ(before.steering_position[i], r.session.output().steering_position[i]);
    }
  }
}
TEST(RelativeMoveSession, ImuLossStopsLatchesAndFreezesEstimate)
{
  SessionRig r; r.operation(); ASSERT_TRUE(r.session.start(request()));
  for (int i = 0; i < 100; ++i) {r.tick();}
  auto before = r.session.state(); const auto target = r.session.output();
  r.feedback.imu_valid = false; r.session.update(0.01, r.feedback);
  EXPECT_EQ(r.session.state().fault, MoveFault::imu_lost);
  EXPECT_EQ(r.session.state().phase, MovePhase::stopping);
  EXPECT_FALSE(r.session.state().estimate_valid);
  EXPECT_GE(r.session.output().wheel_position[0], target.wheel_position[0]);
  r.feedback.imu_valid = true; r.finish();
  EXPECT_EQ(r.session.state().result, MoveResult::faulted);
  EXPECT_EQ(r.session.state().measurement_sequence, before.measurement_sequence);
  EXPECT_DOUBLE_EQ(r.session.state().actual.x_m, before.actual.x_m);
  EXPECT_EQ(r.session.state().quality & 4U, 4U);
  EXPECT_FALSE(r.session.start(request()));
  EXPECT_EQ(r.session.reset_fault(false), 1);
  EXPECT_EQ(r.session.reset_fault(true), 0);
  r.tick(); EXPECT_TRUE(r.session.start(request()));
}
TEST(RelativeMoveSession, ImuMismatchIsReportOnlyAndSuccessIndependentOfResidual)
{
  SessionRig r; r.operation(); ASSERT_TRUE(r.session.start(request()));
  for (int i = 0; i < 4000 && r.session.state().active; ++i) {r.tick(0.01, 0.02);}
  EXPECT_EQ(r.session.state().result, MoveResult::succeeded);
  EXPECT_EQ(r.session.state().fault, MoveFault::none);
  EXPECT_EQ(r.session.state().quality & 3U, 3U);
  EXPECT_TRUE(r.session.state().estimate_valid);
}
TEST(RelativeMoveSession, SlipAndSteeringFaultStopLatch)
{
  for (bool slip : {false, true}) {
    SessionRig r; r.operation(); ASSERT_TRUE(r.session.start(request()));
    for (int i = 0; i < 100; ++i) {r.tick();}
    for (int i = 0; i < 3; ++i) {
      if (slip) {r.feedback.wheel_velocity[0] = 10;}
      else {r.feedback.positions[0].steering_measured_rad = 0.3;}
      r.session.update(0.01, r.feedback);
    }
    EXPECT_EQ(r.session.state().fault, slip ? MoveFault::slip : MoveFault::steering_error);
    EXPECT_EQ(r.session.state().phase, MovePhase::stopping);
    r.finish(); EXPECT_EQ(r.session.state().result, MoveResult::faulted);
  }
}
TEST(RelativeMoveSession, DriveBusAndFeedbackLossInhibitWithoutMeasuredTargetJump)
{
  for (int fault = 0; fault < 3; ++fault) {
    SessionRig r; r.operation(); ASSERT_TRUE(r.session.start(request()));
    for (int i = 0; i < 100; ++i) {r.tick();}
    const auto target = r.session.output();
    r.feedback.positions[0].drive_position_rad -= 0.2;
    if (fault == 0) {r.feedback.drives_ok = false;}
    if (fault == 1) {r.feedback.bus_ok = false;}
    if (fault == 2) {r.feedback.age = 1;}
    r.session.update(0.01, r.feedback);
    EXPECT_EQ(r.session.state().result, MoveResult::faulted);
    EXPECT_TRUE(r.session.output().inhibited);
    EXPECT_EQ(target.wheel_position, r.session.output().wheel_position);
    EXPECT_FALSE(r.session.state().stationary);
    EXPECT_FALSE(r.session.state().estimate_valid);
    EXPECT_EQ(r.session.reset_fault(true), 4);
  }
}
TEST(RelativeMoveSession, TimeoutStopsThenAbortsAndStopFailureDoesNotClaimHold)
{
  SessionRig r; r.operation(); auto req = request(); req.max_duration = 5;
  ASSERT_TRUE(r.session.start(req));
  // Keep feedback stationary away from target: planner completion alone cannot succeed.
  for (int i = 0; i < 1000 && r.session.state().active; ++i) {r.session.update(0.01, r.feedback);}
  EXPECT_EQ(r.session.state().result, MoveResult::faulted);
  EXPECT_TRUE(r.session.output().inhibited);
  EXPECT_NE(r.session.state().fault, MoveFault::none);
  RelativeMoveSession restarted;
  EXPECT_FALSE(restarted.state().active); EXPECT_FALSE(restarted.state().ready);
  EXPECT_EQ(restarted.state().mode, ChassisMode::unknown);
}
TEST(RelativeMoveSession, TimingFailureAndFaultDuringCancelAbort)
{
  SessionRig r; r.operation(); ASSERT_TRUE(r.session.start(request()));
  for (int i = 0; i < 100; ++i) {r.tick();}
  r.session.cancel(); r.feedback.drives_ok = false; r.session.update(0.01, r.feedback);
  EXPECT_EQ(r.session.state().result, MoveResult::faulted);
  SessionRig t; t.operation(); ASSERT_TRUE(t.session.start(request()));
  t.session.update(0.2, t.feedback);
  EXPECT_EQ(t.session.state().fault, MoveFault::execution);
  EXPECT_TRUE(t.session.output().inhibited);
}
TEST(RelativeMoveSession, DeadlineReturnsTimedOutOnlyAfterVerifiedStop)
{
  SessionRig r; r.operation(); auto req = request(); req.max_duration = 3.6;
  ASSERT_TRUE(r.session.start(req));
  for (int i = 0; i < 400 && r.session.state().fault == MoveFault::none; ++i) {
    r.session.update(0.01, r.feedback);
  }
  ASSERT_EQ(r.session.state().fault, MoveFault::timeout);
  EXPECT_TRUE(r.session.state().active);
  EXPECT_EQ(r.session.state().phase, MovePhase::stopping);
  r.finish();
  EXPECT_EQ(r.session.state().result, MoveResult::timed_out);
  EXPECT_EQ(r.session.state().fault, MoveFault::timeout);
  EXPECT_TRUE(r.session.state().stationary);
}
TEST(RelativeMoveSession, StationarySettlingCancelAndPersistentErrorReset)
{
  SessionRig r; r.operation(); ASSERT_TRUE(r.session.start(request({0.01, 0, 0})));
  for (int i = 0; i < 1000 && r.session.state().phase != MovePhase::holding; ++i) {r.tick();}
  ASSERT_EQ(r.session.state().phase, MovePhase::holding);
  ASSERT_TRUE(r.session.state().active);
  r.session.cancel(); r.finish();
  EXPECT_EQ(r.session.state().result, MoveResult::canceled);
  r.feedback.positions[0].steering_measured_rad += 0.3;
  for (int i = 0; i < 10; ++i) {r.session.update(0.01, r.feedback);}
  EXPECT_EQ(r.session.state().fault, MoveFault::steering_error);
  EXPECT_EQ(r.session.reset_fault(true), 4);
}
TEST(RelativeMoveSession, ImuJumpIsInvalidAndBusFaultPreservesFirstLatch)
{
  SessionRig r; r.operation(); ASSERT_TRUE(r.session.start(request()));
  for (int i = 0; i < 100; ++i) {r.tick();}
  r.feedback.imu_yaw += 0.3; r.session.update(0.01, r.feedback);
  EXPECT_EQ(r.session.state().fault, MoveFault::imu_lost);
  EXPECT_FALSE(r.session.state().estimate_valid);
  r.feedback.bus_ok = false; r.session.update(0.01, r.feedback);
  EXPECT_EQ(r.session.state().result, MoveResult::faulted);
  EXPECT_EQ(r.session.state().fault, MoveFault::imu_lost);
}
TEST(RelativeMoveSession, ExtremeFiniteFeedbackCannotEscapeNoexceptCore)
{
  auto config = session_config(); config.motion.modules[0].location.x = 1e308;
  RelativeMoveSession invalid; EXPECT_FALSE(invalid.configure(config));
  SessionRig imu; imu.operation();
  imu.feedback.imu_yaw = 1e308; imu.session.update(0.01, imu.feedback);
  ASSERT_TRUE(imu.session.start(request()));
  imu.feedback.imu_yaw = -1e308; imu.session.update(0.01, imu.feedback);
  EXPECT_EQ(imu.session.state().fault, MoveFault::imu_lost);
  EXPECT_FALSE(imu.session.state().estimate_valid);
  SessionRig wheel;
  RelativeMoveSession overflow;
  config = session_config();
  for (auto & module : config.motion.modules) {module.wheel_radius_m = 10;}
  ASSERT_TRUE(overflow.configure(config));
  wheel.feedback.drive_mode.fill(8);
  for (int i = 0; i < 5; ++i) {overflow.update(0.01, wheel.feedback);}
  ASSERT_TRUE(overflow.start(request()));
  wheel.feedback.wheel_velocity.fill(std::numeric_limits<double>::max());
  overflow.update(0.01, wheel.feedback);
  EXPECT_EQ(overflow.state().result, MoveResult::faulted);
  EXPECT_TRUE(overflow.output().inhibited);
}
