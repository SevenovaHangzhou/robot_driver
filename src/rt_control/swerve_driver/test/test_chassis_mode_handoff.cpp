#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include "swerve_driver/chassis_mode_handoff.hpp"

using namespace swerve_driver;
namespace
{
// Synthetic offline/mock-only values. These are not robot calibration or status predicates.
ChassisHandoffConfig mock_config()
{
  ChassisHandoffConfig c;
  c.max_wheel_velocity.fill(2); c.max_wheel_acceleration.fill(4);
  c.position_tolerance.fill(0.001);
  c.stationary_wheel_velocity = c.stationary_steering_velocity = 0.01;
  c.stationary_dwell = 0.02; c.feedback_timeout = 0.05; c.command_timeout = 0.1;
  c.max_update_period = 0.02; c.stop_timeout = 1; c.switch_timeout = 0.3;
  c.enabled = {0x000f, 0x0005}; c.csv_ready = {0x003f, 0x0025};
  c.csp_ready = {0x103f, 0x1025};
  return c;
}
struct Rig
{
  ChassisModeHandoff core;
  HandoffFeedback feedback{};
  HandoffAcknowledgements ack{};
  double now{1};
  Rig()
  {
    EXPECT_TRUE(core.configure(mock_config(), 100));
    for (std::size_t i = 0; i < 4; ++i) {
      feedback[i] = {10 + static_cast<double>(i), 0, 0, now, 1, 0x1025, 9, true};
    }
    EXPECT_TRUE(core.start_navigation(now, feedback, {}));
  }
  // Explicitly models a completed cyclic send, independently of 6061 and observation.
  void sent()
  {
    for (std::size_t i = 0; i < 4; ++i) {
      ack[i] = {core.output().write_sequence, feedback[i].drive_sequence};
    }
  }
  void tick(bool fresh = true)
  {
    now += 0.01;
    if (fresh) {
      for (auto & d : feedback) {d.sample_time = now; ++d.drive_sequence;}
    }
    core.update(now, feedback, ack);
  }
  void navigation()
  {
    sent();
    for (int i = 0; i < 4; ++i) {tick();}
    EXPECT_EQ(core.phase(), HandoffPhase::confirm_csv);
    sent(); tick();
    EXPECT_TRUE(core.navigation_open());
  }
  HandoffNavigationCommand command(double velocity) const
  {
    HandoffNavigationCommand c;
    c.wheel_velocity.fill(velocity); c.generation = core.generation(); c.received_at = now;
    return c;
  }
  void seed_csp()
  {
    EXPECT_TRUE(core.begin_operation(now));
    for (int i = 0; i < 4; ++i) {tick();}
    EXPECT_EQ(core.phase(), HandoffPhase::seed_csp);
  }
  void operation()
  {
    seed_csp(); sent(); tick();
    EXPECT_EQ(core.phase(), HandoffPhase::confirm_csp);
    sent(); for (auto & d : feedback) {d.mode = 8;}
    tick(); EXPECT_TRUE(core.operation_ready());
  }
};
}
TEST(ChassisModeHandoff, RequiresExplicitFiniteCalibrationAndStatusConfiguration)
{
  ChassisModeHandoff c;
  EXPECT_FALSE(c.configure({}, 100));
  EXPECT_FALSE(c.configure(mock_config(), 0));
  auto config = mock_config(); config.enabled.mask = 0;
  EXPECT_FALSE(c.configure(config, 100));
  config = mock_config(); config.csp_ready.value &= static_cast<uint16_t>(~0x1000U);
  EXPECT_FALSE(c.configure(config, 100));
  config = mock_config(); config.max_wheel_acceleration[0] = std::numeric_limits<double>::max();
  config.max_update_period = 2; config.switch_timeout = config.stop_timeout = 20;
  EXPECT_FALSE(c.configure(config, 100));
  config = mock_config(); config.max_wheel_velocity[0] = std::numeric_limits<double>::max();
  EXPECT_FALSE(c.configure(config, 100));
  config = mock_config(); config.max_wheel_acceleration[0] = 0.001;
  EXPECT_FALSE(c.configure(config, 100)); // Cannot stop within explicit deadline.
  EXPECT_TRUE(c.configure(mock_config(), 100));
  EXPECT_FALSE(c.configure(mock_config(), 100));
}
TEST(ChassisModeHandoff, StartupNeedsExplicitZeroReferenceFreshStationaryAndSendAcknowledgement)
{
  ChassisModeHandoff c; ASSERT_TRUE(c.configure(mock_config(), 200));
  Rig r;
  HandoffWheels moving{{0, 0, 0, 1}};
  EXPECT_FALSE(c.start_navigation(r.now, r.feedback, moving));
  auto f = r.feedback; f[3].steering_velocity = 0.1;
  EXPECT_FALSE(c.start_navigation(r.now, f, {}));
  f = r.feedback; f[3].mode = 8;
  EXPECT_FALSE(c.start_navigation(r.now, f, {}));
  ASSERT_TRUE(c.start_navigation(r.now, r.feedback, {}));
  EXPECT_FALSE(c.navigation_open());
  for (int i = 0; i < 5; ++i) {
    r.tick();
  }
  EXPECT_EQ(r.core.phase(), HandoffPhase::seed_csv); // Readback 9 cannot acknowledge send.
  EXPECT_FALSE(r.core.output().write_mode);
}
TEST(ChassisModeHandoff, ExactPositionSeedMustBeSentBeforeModeRequestAndNewReadbackBeforeReady)
{
  Rig r; r.navigation(); r.seed_csp();
  const auto seed = r.core.output();
  ASSERT_TRUE(seed.write_position); EXPECT_TRUE(seed.write_velocity);
  EXPECT_FALSE(seed.write_mode);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(seed.wheel_position[i], r.feedback[i].wheel_position);
    EXPECT_DOUBLE_EQ(seed.wheel_velocity[i], 0);
  }
  for (int i = 0; i < 5; ++i) {
    r.tick();
  }
  EXPECT_EQ(r.core.output().write_sequence, seed.write_sequence);
  EXPECT_FALSE(r.core.output().write_mode);
  r.sent(); r.ack[3].write_sequence = 0; r.tick();
  EXPECT_EQ(r.core.phase(), HandoffPhase::seed_csp);
  r.sent(); r.tick();
  EXPECT_EQ(r.core.phase(), HandoffPhase::confirm_csp);
  EXPECT_EQ(r.core.output().requested_mode, 8);
  EXPECT_GT(r.core.output().write_sequence, seed.write_sequence);
  EXPECT_EQ(r.core.output().wheel_position, seed.wheel_position);
  for (auto & d : r.feedback) {
    d.mode = 8;
  }
  r.tick(); EXPECT_FALSE(r.core.operation_ready()); // Old seed ACK insufficient.
  r.sent(); r.tick(false); EXPECT_FALSE(r.core.operation_ready()); // Same pre-send feedback.
  r.tick(); EXPECT_TRUE(r.core.operation_ready());
  EXPECT_FALSE(r.core.output().write_position); // Relative executor now owns target.
}
TEST(ChassisModeHandoff, StopsWithBoundedReferenceContinuityThenWaitsForMeasuredWheelAndSteering)
{
  Rig r; r.navigation(); r.tick();
  ASSERT_TRUE(r.core.accept_navigation(r.command(1), r.now));
  for (int i = 0; i < 5; ++i) {
    r.tick();
  }
  const auto before = r.core.output().wheel_velocity;
  ASSERT_GT(before[0], 0);
  ASSERT_TRUE(r.core.begin_operation(r.now));
  EXPECT_EQ(r.core.output().wheel_velocity, before); // No immediate zero jump.
  EXPECT_FALSE(r.core.accept_navigation(r.command(1), r.now));
  r.feedback[0].wheel_velocity = 0.2;
  double previous = before[0];
  for (int i = 0; i < 10; ++i) {
    r.tick(); const double current = r.core.output().wheel_velocity[0];
    EXPECT_GE(current, 0); EXPECT_LE(current, previous);
    EXPECT_LE(previous - current, 0.04000000001); previous = current;
  }
  EXPECT_DOUBLE_EQ(previous, 0);
  EXPECT_EQ(r.core.phase(), HandoffPhase::stopping_csv);
  r.feedback[0].wheel_velocity = 0; r.feedback[2].steering_velocity = 0.2;
  for (int i = 0; i < 4; ++i) {
    r.tick();
  }
  EXPECT_EQ(r.core.phase(), HandoffPhase::stopping_csv);
  r.feedback[2].steering_velocity = 0;
  for (int i = 0; i < 4; ++i) {
    r.tick();
  }
  EXPECT_EQ(r.core.phase(), HandoffPhase::seed_csp);
}
TEST(ChassisModeHandoff, StationaryDwellResetsAndCachedSamplesDoNotAdvanceIt)
{
  Rig r; r.navigation(); ASSERT_TRUE(r.core.begin_operation(r.now));
  r.tick(); r.tick();
  r.feedback[1].steering_velocity = 0.1; r.tick();
  r.feedback[1].steering_velocity = 0; r.tick();
  r.tick(false); r.tick(false);
  EXPECT_EQ(r.core.phase(), HandoffPhase::stopping_csv);
  r.tick(); EXPECT_EQ(r.core.phase(), HandoffPhase::seed_csp);
}
TEST(ChassisModeHandoff, PartialFourDriveSwitchWaitsAndTimesOutWithoutAutomaticResumption)
{
  Rig r; r.navigation(); r.seed_csp(); r.sent(); r.tick(); r.sent();
  for (std::size_t i = 0; i < 3; ++i) {
    r.feedback[i].mode = 8;
  }
  r.tick(); EXPECT_FALSE(r.core.operation_ready());
  for (int i = 0; i < 35; ++i) {
    r.tick();
  }
  EXPECT_EQ(r.core.cause(), HandoffCause::switch_timeout);
  EXPECT_TRUE(r.core.output().inhibited);
  r.feedback[3].mode = 8; r.tick();
  EXPECT_FALSE(r.core.operation_ready()); EXPECT_FALSE(r.core.begin_operation(r.now));
}
TEST(ChassisModeHandoff, TargetValidAndConfiguredModeSpecificStateRequiredForAllFour)
{
  Rig r; r.navigation(); r.seed_csp(); r.sent(); r.tick(); r.sent();
  for (auto & d : r.feedback) {
    d.mode = 8;
  }
  r.feedback[3].status_word = 0x0025; r.tick(); EXPECT_FALSE(r.core.operation_ready());
  r.feedback[3].status_word = 0x1005; r.tick(); EXPECT_FALSE(r.core.operation_ready());
  r.feedback[3].status_word = 0x1025; r.tick(); EXPECT_TRUE(r.core.operation_ready());
  r.feedback[0].status_word = 0x0025; r.tick();
  EXPECT_EQ(r.core.cause(), HandoffCause::drive);
}
TEST(ChassisModeHandoff, ReverseWritesZeroBeforeCsvRequestAndRequiresFreshGenerationAfterCompletion)
{
  Rig r; r.navigation(); r.tick(); const auto old_command = r.command(1);
  r.operation();
  HandoffWheels held{};
  for (std::size_t i = 0; i < 4; ++i) {
    held[i] = r.feedback[i].wheel_position + 0.0001;
  }
  ASSERT_TRUE(r.core.begin_navigation(r.now, held));
  const auto zero = r.core.output();
  EXPECT_EQ(zero.wheel_position, held); EXPECT_TRUE(zero.write_position);
  EXPECT_TRUE(zero.write_velocity); EXPECT_FALSE(zero.write_mode);
  EXPECT_EQ(zero.wheel_velocity, HandoffWheels{});
  EXPECT_FALSE(r.core.accept_navigation(old_command, r.now));
  auto during = r.command(1); r.tick();
  EXPECT_EQ(r.core.phase(), HandoffPhase::seed_csv);
  r.sent(); r.tick(); EXPECT_EQ(r.core.output().requested_mode, 9);
  EXPECT_GT(r.core.output().write_sequence, zero.write_sequence);
  for (auto & d : r.feedback) {
    d.mode = 9;
  }
  r.tick(); EXPECT_FALSE(r.core.navigation_open());
  r.sent(); r.tick(); ASSERT_TRUE(r.core.navigation_open());
  EXPECT_FALSE(r.core.accept_navigation(old_command, r.now));
  EXPECT_FALSE(r.core.accept_navigation(during, r.now));
  during.generation = r.core.generation();
  EXPECT_FALSE(r.core.accept_navigation(during, r.now)); // Even retagged old receipt fails.
  EXPECT_FALSE(r.core.accept_navigation(r.command(1), r.now)); // Completion-time receipt too early.
  r.tick(); EXPECT_EQ(r.core.output().wheel_velocity, HandoffWheels{});
  ASSERT_TRUE(r.core.accept_navigation(r.command(1), r.now));
  r.tick(); EXPECT_GT(r.core.output().wheel_velocity[0], 0);
}
TEST(ChassisModeHandoff, ReverseRequiresStoppedExecutorAndPreservesHeldTarget)
{
  Rig r; r.navigation(); r.operation();
  auto held = r.core.output().wheel_position;
  auto invalid = held; invalid[0] += 0.1;
  EXPECT_FALSE(r.core.begin_navigation(r.now, invalid));
  r.feedback[0].wheel_velocity = 0.1; r.tick();
  EXPECT_FALSE(r.core.begin_navigation(r.now, held));
  r.feedback[0].wheel_velocity = 0;
  for (int i = 0; i < 4; ++i) {
    r.tick();
  }
  EXPECT_TRUE(r.core.begin_navigation(r.now, held));
  EXPECT_EQ(r.core.output().wheel_position, held);
}
TEST(ChassisModeHandoff, MovingAfterSeedInhibitsWithoutReseedingOrImmediateZeroClaim)
{
  Rig r; r.navigation(); r.seed_csp(); const auto seed = r.core.output().wheel_position;
  r.feedback[3].wheel_position += 0.01; r.tick();
  EXPECT_EQ(r.core.cause(), HandoffCause::seed_moved);
  EXPECT_EQ(r.core.output().wheel_position, seed);
  EXPECT_TRUE(r.core.output().inhibited); EXPECT_FALSE(r.core.output().write_position);
}
TEST(ChassisModeHandoff, FeedbackLossAndMalformedNumbersInhibitAndLatchFirstCause)
{
  for (int injection = 0; injection < 7; ++injection) {
    Rig r; r.navigation(); r.tick();
    ASSERT_TRUE(r.core.accept_navigation(r.command(1), r.now)); r.tick();
    const auto velocity = r.core.output().wheel_velocity;
    switch (injection) {
      case 0: r.feedback[0].sample_time = r.now - 1; break;
      case 1: r.feedback[0].wheel_position = std::numeric_limits<double>::quiet_NaN(); break;
      case 2: r.feedback[0].steering_velocity = std::numeric_limits<double>::infinity(); break;
      case 3: r.feedback[0].healthy = false; break;
      case 4: r.feedback[0].status_word = 0; break;
      case 5: r.feedback[0].mode = 0; break;
      case 6: r.feedback[0].drive_sequence = 0; break;
    }
    r.tick(false); EXPECT_TRUE(r.core.output().inhibited);
    EXPECT_EQ(r.core.output().wheel_velocity, velocity); // Not a claimed controlled stop.
    const auto cause = r.core.cause(); EXPECT_NE(cause, HandoffCause::none);
    r.core.cancel(); r.core.deactivate(); EXPECT_EQ(r.core.cause(), cause);
    EXPECT_FALSE(r.core.navigation_open());
  }
}
TEST(ChassisModeHandoff, TimeRegressionAndWatchdogOverrunInhibit)
{
  for (double dt : {-0.01, 0.0, 0.03, std::numeric_limits<double>::quiet_NaN()}) {
    Rig r; r.navigation(); r.core.update(r.now + dt, r.feedback, r.ack);
    EXPECT_EQ(r.core.cause(), HandoffCause::timing);
    EXPECT_TRUE(r.core.output().inhibited);
  }
}
TEST(ChassisModeHandoff, MissingSendAndUnstoppableMeasurementsHaveDistinctDeadlines)
{
  Rig seed; seed.navigation(); seed.seed_csp();
  for (int i = 0; i < 35; ++i) {
    seed.tick();
  }
  EXPECT_EQ(seed.core.cause(), HandoffCause::switch_timeout);
  Rig stop; stop.navigation(); ASSERT_TRUE(stop.core.begin_operation(stop.now));
  stop.feedback[0].wheel_velocity = 0.1;
  for (int i = 0; i < 105; ++i) {
    stop.tick();
  }
  EXPECT_EQ(stop.core.cause(), HandoffCause::stop_timeout);
}
TEST(ChassisModeHandoff, RejectsMalformedFutureExpiredDuplicateAndWrongGenerationCommands)
{
  Rig r; r.navigation(); r.tick();
  auto c = r.command(1); c.generation -= 1; EXPECT_FALSE(r.core.accept_navigation(c, r.now));
  c = r.command(1); c.received_at += 1; EXPECT_FALSE(r.core.accept_navigation(c, r.now));
  c = r.command(1); c.received_at -= 1; EXPECT_FALSE(r.core.accept_navigation(c, r.now));
  c = r.command(1); c.wheel_velocity[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(r.core.accept_navigation(c, r.now));
  EXPECT_FALSE(r.core.accept_navigation(r.command(3), r.now));
  c = r.command(1); EXPECT_TRUE(r.core.accept_navigation(c, r.now));
  EXPECT_FALSE(r.core.accept_navigation(c, r.now));
  for (int i = 0; i < 25; ++i) {
    r.tick();
  }
  EXPECT_EQ(r.core.output().wheel_velocity, HandoffWheels{});
  EXPECT_TRUE(r.core.navigation_open());
  EXPECT_FALSE(r.core.accept_navigation(c, r.now));
}
TEST(ChassisModeHandoff, CancelDeactivateAndRestartNeverReplayOrReenable)
{
  for (bool cancel : {false, true}) {
    Rig r; r.navigation(); r.seed_csp();
    if (cancel) {r.core.cancel();} else {r.core.deactivate();}
    EXPECT_EQ(r.core.cause(), cancel ? HandoffCause::canceled : HandoffCause::deactivated);
    EXPECT_TRUE(r.core.output().inhibited); r.sent(); r.tick();
    EXPECT_FALSE(r.core.operation_ready());
    EXPECT_FALSE(r.core.start_navigation(r.now, r.feedback, {}));
    EXPECT_FALSE(r.core.configure(mock_config(), 200));
    ChassisModeHandoff restarted; EXPECT_TRUE(restarted.configure(mock_config(), 200));
    EXPECT_TRUE(restarted.output().inhibited); EXPECT_FALSE(restarted.navigation_open());
    EXPECT_FALSE(restarted.accept_navigation(r.command(1), r.now));
  }
}
TEST(ChassisModeHandoff, ImpossibleAcknowledgementsAndPrematureReadbackFailClosed)
{
  Rig future; future.navigation(); future.seed_csp(); future.sent();
  ++future.ack[2].write_sequence; future.tick();
  EXPECT_EQ(future.core.cause(), HandoffCause::acknowledgement);
  Rig premature; premature.navigation(); premature.seed_csp();
  premature.feedback[2].mode = 8; premature.tick();
  EXPECT_EQ(premature.core.cause(), HandoffCause::mode);
  Rig invalid; invalid.navigation(); invalid.seed_csp(); invalid.sent();
  invalid.ack[1].feedback_sequence_at_send = 0; invalid.tick();
  EXPECT_EQ(invalid.core.cause(), HandoffCause::acknowledgement);
}
TEST(ChassisModeHandoff, BothVelocitySignsAndExtremeFiniteSeedArithmeticRemainDefined)
{
  Rig r; r.navigation(); r.tick(); auto c = r.command(-1); c.wheel_velocity[2] = 1;
  ASSERT_TRUE(r.core.accept_navigation(c, r.now)); r.tick();
  EXPECT_LT(r.core.output().wheel_velocity[0], 0); EXPECT_GT(r.core.output().wheel_velocity[2], 0);
  ASSERT_TRUE(r.core.begin_operation(r.now));
  for (int i = 0; i < 6; ++i) {
    r.tick();
  }
  EXPECT_EQ(r.core.output().wheel_velocity, HandoffWheels{});
  Rig extreme; extreme.navigation();
  extreme.feedback[0].wheel_position = std::numeric_limits<double>::max(); extreme.tick();
  extreme.seed_csp();
  extreme.feedback[0].wheel_position = -std::numeric_limits<double>::max(); extreme.tick();
  EXPECT_EQ(extreme.core.cause(), HandoffCause::seed_moved);
}
TEST(ChassisModeHandoff, ReversePartialModeConfirmationTimesOutWithAdmissionClosed)
{
  Rig r; r.navigation(); r.operation();
  ASSERT_TRUE(r.core.begin_navigation(r.now, r.core.output().wheel_position));
  r.sent(); r.tick(); r.sent();
  for (std::size_t i = 0; i < 3; ++i) {
    r.feedback[i].mode = 9;
  }
  r.tick(); EXPECT_EQ(r.core.phase(), HandoffPhase::confirm_csv);
  for (int i = 0; i < 35; ++i) {
    r.tick();
  }
  EXPECT_EQ(r.core.cause(), HandoffCause::switch_timeout);
  EXPECT_FALSE(r.core.navigation_open());
}
TEST(ChassisModeHandoff, LateSeedAcknowledgementDoesNotRestartSwitchDeadline)
{
  Rig r; r.navigation(); r.seed_csp();
  for (int i = 0; i < 23; ++i) {
    r.tick();
  }
  r.sent(); r.tick(); ASSERT_EQ(r.core.phase(), HandoffPhase::confirm_csp);
  r.sent();
  for (int i = 0; i < 8; ++i) {
    r.tick();
  }
  EXPECT_EQ(r.core.cause(), HandoffCause::switch_timeout);
}
TEST(ChassisModeHandoff, StaleFeedbackBlocksRequestsAndLatchesDuringSwitch)
{
  Rig r; r.navigation();
  EXPECT_FALSE(r.core.begin_operation(r.now + 0.03));
  EXPECT_TRUE(r.core.navigation_open());
  r.seed_csp();
  for (int i = 0; i < 6; ++i) {
    r.tick(false);
  }
  EXPECT_EQ(r.core.cause(), HandoffCause::feedback);
  EXPECT_TRUE(r.core.output().inhibited);
}
TEST(ChassisModeHandoff, GenerationExhaustionFailsClosedInsteadOfReusingOldCommands)
{
  ChassisModeHandoff core;
  ASSERT_TRUE(core.configure(mock_config(), std::numeric_limits<uint64_t>::max() - 1));
  Rig r;
  ASSERT_TRUE(core.start_navigation(r.now, r.feedback, {}));
  HandoffAcknowledgements ack{};
  for (int iteration = 0; iteration < 6; ++iteration) {
    for (std::size_t i = 0; i < 4; ++i) {
      ack[i] = {core.output().write_sequence, r.feedback[i].drive_sequence};
    }
    r.tick(); core.update(r.now, r.feedback, ack);
  }
  ASSERT_TRUE(core.navigation_open());
  EXPECT_FALSE(core.begin_operation(r.now));
  EXPECT_EQ(core.cause(), HandoffCause::sequence_exhausted);
  EXPECT_TRUE(core.output().inhibited);
}
