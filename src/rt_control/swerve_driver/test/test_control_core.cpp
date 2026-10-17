#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "swerve_driver/control_core.hpp"

namespace swerve_driver
{
namespace
{
CoreConfig synthetic_config()
{
  CoreConfig c;
  c.locations = {{{0.5, 0.4}, {0.5, -0.4}, {-0.5, 0.4}, {-0.5, -0.4}}};
  c.wheel_radius.fill(0.1);
  c.steering_min.fill(-kPi);
  c.steering_max.fill(kPi);
  c.steering_limit_margin.fill(0.05);
  c.steering_limit_tolerance.fill(0.01);
  c.max_linear_speed = 2.0;
  c.max_angular_speed = 3.0;
  c.max_wheel_speed = 2.0;
  c.max_wheel_acceleration = 10.0;
  c.velocity_deadband = 0.001;
  c.max_encoder_difference = 0.2;
  c.max_update_period = 0.02;
  c.slip_residual_threshold = 0.1;
  c.setpoint = {0.3, 0.1, 5.0};
  return c;
}

class ControlCoreTest : public testing::Test
{
protected:
  CoreConfig config{synthetic_config()};
  ControlCore core{config};
  ModuleFeedbackArray feedback{};
  void SetUp() override
  {
    for (auto & module : feedback) {module.valid = true; module.enabled = true;}
    ASSERT_TRUE(core.activate(feedback));
  }
};

TEST(ControlConfiguration, UnspecifiedGeometryCannotActivate)
{
  EXPECT_THROW(ControlCore{CoreConfig{}}, std::invalid_argument);
  auto config = synthetic_config();
  config.locations.fill({0.0, 0.0});
  EXPECT_THROW(ControlCore{config}, std::invalid_argument);
  config = synthetic_config();
  config.steering_max[2] = 0.5;
  config.steering_min[2] = -0.5;
  EXPECT_THROW(ControlCore{config}, std::invalid_argument);
}

TEST_F(ControlCoreTest, CommandsOnlyCspPositionsAndCsvVelocities)
{
  const auto result = core.update(feedback, {1.0, 0.0, 0.0}, true, 0.004);
  EXPECT_EQ(result.status, ControlStatus::running);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(result.steering_position[i], 0.0);
    EXPECT_NEAR(result.drive_velocity[i], 0.4, 1e-12);
  }
}

TEST_F(ControlCoreTest, ExternalAngleGatesDriveWithoutCorrectingMotorZero)
{
  for (auto & module : feedback) {module.steering_angle = 0.15;}
  for (int cycle = 0; cycle < 100; ++cycle) {
    const auto result = core.update(feedback, {1.0, 0.0, 0.0}, true, 0.004);
    if (cycle > 10) {
      for (const auto target : result.steering_position) {EXPECT_NEAR(target, 0.0, 1e-12);}
    }
  }
  for (auto & module : feedback) {
    module.steering_angle = 0.5;
    module.steering_position = 0.5;
  }
  const auto result = core.update(feedback, {1.0, 0.0, 0.0}, true, 0.004);
  EXPECT_EQ(result.status, ControlStatus::alignment_gated);
  for (const auto speed : result.drive_velocity) {EXPECT_DOUBLE_EQ(speed, 0.0);}
}

TEST_F(ControlCoreTest, AnyInvalidFeedbackStopsAllDriveWheelsAndHoldsLastTargets)
{
  core.update(feedback, {1.0, 0.0, 0.0}, true, 0.004);
  feedback[2].valid = false;
  feedback[0].steering_position = 0.08;
  const auto result = core.update(feedback, {1.0, 0.0, 0.0}, true, 0.004);
  EXPECT_EQ(result.status, ControlStatus::feedback_fault);
  EXPECT_DOUBLE_EQ(result.steering_position[0], 0.0);
  for (const auto speed : result.drive_velocity) {EXPECT_DOUBLE_EQ(speed, 0.0);}
}

TEST_F(ControlCoreTest, RejectedSteeringJumpCannotPolluteOdometryBeforeStop)
{
  for (auto & module : feedback) {module.wheel_position = 1.0;}
  feedback[2].wheel_position = 10.0;
  feedback[2].steering_angle = 2.0;

  const auto result = core.update(feedback, {}, false, 0.004);

  EXPECT_EQ(result.status, ControlStatus::steering_hold_fault);
  EXPECT_NEAR(result.pose.x_m, 0.1, 1e-12);
  EXPECT_NEAR(result.pose.y_m, 0.0, 1e-12);
}

TEST_F(ControlCoreTest, MotorEncoderMismatchRejectsActivationAndCommands)
{
  feedback[0].steering_angle = 2.0 * kPi;
  EXPECT_FALSE(core.activate(feedback));
  EXPECT_EQ(core.update(feedback, {}, true, 0.004).status, ControlStatus::inactive);
}

TEST_F(ControlCoreTest, StaleCommandsAndBadTimingNeverProduceMotion)
{
  EXPECT_EQ(core.update(feedback, {1.0, 0.0, 0.0}, false, 0.004).status,
    ControlStatus::command_timeout);
  EXPECT_EQ(core.update(feedback, {1.0, 0.0, 0.0}, true, 1.0).status,
    ControlStatus::invalid_command);
  EXPECT_EQ(core.update(feedback, {std::numeric_limits<double>::quiet_NaN(), 0, 0},
      true, 0.004).status, ControlStatus::invalid_command);
}

TEST_F(ControlCoreTest, OdometryUsesWheelMotionAndExternalSteering)
{
  for (auto & module : feedback) {
    module.steering_position = 1.5;
    module.steering_angle = kPi / 2.0;
  }
  ASSERT_TRUE(core.activate(feedback));
  for (auto & module : feedback) {
    module.wheel_position = 1.0;
    module.wheel_velocity = 2.0;
  }
  const auto result = core.update(feedback, {}, false, 0.004);
  EXPECT_NEAR(result.pose.x_m, 0.0, 1e-12);
  EXPECT_NEAR(result.pose.y_m, 0.1, 1e-12);
  EXPECT_NEAR(result.measured_twist.vy_mps, 0.2, 1e-12);
}

TEST_F(ControlCoreTest, SlipIsExcludedWithoutRecoveryCatchUp)
{
  for (auto & module : feedback) {
    module.wheel_position = 1.0;
    module.wheel_velocity = 10.0;
  }
  feedback[2].wheel_position = 3.0;
  feedback[2].wheel_velocity = 30.0;

  auto result = core.update(feedback, {}, false, 0.004);

  EXPECT_TRUE(result.slip_detected);
  EXPECT_TRUE(result.slipping_modules[2]);
  EXPECT_EQ(result.valid_modules, 3U);
  EXPECT_NEAR(result.measured_twist.vx_mps, 1.0, 1e-12);
  EXPECT_NEAR(result.pose.x_m, 0.1, 1e-12);

  for (auto & module : feedback) {
    module.wheel_position += 1.0;
    module.wheel_velocity = 10.0;
  }
  result = core.update(feedback, {}, false, 0.004);

  EXPECT_FALSE(result.slip_detected);
  EXPECT_EQ(result.valid_modules, 4U);
  EXPECT_NEAR(result.pose.x_m, 0.2, 1e-12);
}

TEST_F(ControlCoreTest, ActivationAndDeactivationDoNotReplayTargets)
{
  core.update(feedback, {1, 0, 0}, true, 0.004);
  core.deactivate();
  EXPECT_EQ(core.update(feedback, {1, 0, 0}, true, 0.004).status, ControlStatus::inactive);
  ASSERT_TRUE(core.activate(feedback));
  const auto result = core.update(feedback, {}, false, 0.004);
  for (const auto speed : result.drive_velocity) {EXPECT_DOUBLE_EQ(speed, 0.0);}
}

TEST_F(ControlCoreTest, ZeroVelocityHoldsMotorPositionDespiteSensorDifference)
{
  for (auto & module : feedback) {module.steering_angle = 0.15;}
  const auto result = core.update(feedback, {}, true, 0.004);
  for (const auto target : result.steering_position) {EXPECT_DOUBLE_EQ(target, 0.0);}
}

TEST_F(ControlCoreTest, InvalidMotorPositionHoldsLastTargetWithoutResampling)
{
  const auto moving = core.update(feedback, {0.0, 1.0, 0.0}, true, 0.004);
  ASSERT_GT(moving.steering_position[0], 0.0);
  feedback[0].steering_position = std::numeric_limits<double>::quiet_NaN();
  const auto stopped = core.update(feedback, {}, true, 0.004);
  EXPECT_DOUBLE_EQ(stopped.steering_position[0], moving.steering_position[0]);
}

TEST_F(ControlCoreTest, ImuFallbackAndRecoveryKeepHeadingContinuous)
{
  auto result = core.update(feedback, {}, false, 0.004, YawSample{1.0, 0.2});
  EXPECT_FALSE(result.imu_fallback);
  const auto first = result.pose.heading_rad;
  result = core.update(feedback, {}, false, 0.004, YawSample{1.1, 0.2});
  EXPECT_NEAR(result.pose.heading_rad - first, 0.1, 1e-12);
  result = core.update(feedback, {}, false, 0.004);
  EXPECT_TRUE(result.imu_fallback);
  const auto fallback = result.pose.heading_rad;
  result = core.update(feedback, {}, false, 0.004, YawSample{2.0, 0.1});
  EXPECT_NEAR(result.pose.heading_rad, fallback, 1e-12);
}

TEST_F(ControlCoreTest, FeedbackOutsideSafeSteeringTravelCannotProduceMotion)
{
  feedback[0].steering_position = kPi;
  const auto result = core.update(feedback, {0, 1, 0}, true, 0.004);
  EXPECT_EQ(result.status, ControlStatus::steering_hold_fault);
  for (const auto speed : result.drive_velocity) {EXPECT_EQ(speed, 0.0);}
}

TEST_F(ControlCoreTest, PiBoundaryCommandReversesWheelWithoutCrossingMechanicalLimit)
{
  core.deactivate();
  auto fast = config;
  fast.setpoint.maximum_steering_slew_radps = 1000.0;
  fast.steering_limit_margin.fill(0.0);
  ControlCore bounded{fast};
  for (auto & module : feedback) {
    module.steering_position = 179.0 * kPi / 180.0;
    module.steering_angle = module.steering_position;
  }
  ASSERT_TRUE(bounded.activate(feedback));
  const double heading{-179.0 * kPi / 180.0};

  const auto result = bounded.update(
    feedback, {std::cos(heading), std::sin(heading), 0.0}, true, 0.004);

  EXPECT_EQ(result.status, ControlStatus::alignment_gated);
  for (std::size_t i{0U}; i < result.steering_position.size(); ++i) {
    EXPECT_NEAR(result.steering_position[i], kPi / 180.0, 1e-12);
    EXPECT_DOUBLE_EQ(result.drive_velocity[i], 0.0);
  }
}

TEST_F(ControlCoreTest, ZeroCommandDoesNotFeedMeasurementJitterBackIntoHoldTarget)
{
  for (auto & module : feedback) {
    module.steering_position = module.steering_angle = 0.35;
  }
  ASSERT_TRUE(core.activate(feedback));
  for (int cycle = 0; cycle < 1000; ++cycle) {
    for (auto & module : feedback) {
      module.steering_position = module.steering_angle = 0.35 + (cycle % 2 ? 0.004 : -0.004);
    }
    const auto output = core.update(feedback, {}, true, 0.004);
    EXPECT_EQ(output.status, ControlStatus::running);
    for (size_t i = 0; i < 4; ++i) {
      EXPECT_DOUBLE_EQ(output.steering_position[i], 0.35);
      EXPECT_DOUBLE_EQ(output.drive_velocity[i], 0.0);
    }
  }
}

TEST_F(ControlCoreTest, CommandTimeoutKeepsTheLastTargetDespiteFeedbackNoise)
{
  const auto commanded = core.update(feedback, {0.0, 1.0, 0.0}, true, 0.004);
  ASSERT_GT(commanded.steering_position[0], 0.0);
  for (int cycle = 0; cycle < 50; ++cycle) {
    for (size_t i = 0; i < 4; ++i) {
      feedback[i].steering_position = feedback[i].steering_angle =
        commanded.steering_position[i] + (cycle % 2 ? 0.004 : -0.004);
    }
    const auto output = core.update(feedback, {0.0, 1.0, 0.0}, false, 0.004);
    EXPECT_EQ(output.status, ControlStatus::command_timeout);
    EXPECT_EQ(output.steering_position, commanded.steering_position);
    for (auto velocity : output.drive_velocity) {
      EXPECT_DOUBLE_EQ(velocity, 0.0);
    }
  }
}

TEST_F(ControlCoreTest, MotionToStopKeepsCommandedTargetRatherThanMeasuredPosition)
{
  const auto moving = core.update(feedback, {0.0, 1.0, 0.0}, true, 0.004);
  ASSERT_NE(moving.steering_position[0], feedback[0].steering_position);
  auto stopped = core.update(feedback, {}, true, 0.004);
  EXPECT_EQ(stopped.steering_position, moving.steering_position);
  for (auto velocity : stopped.drive_velocity) {
    EXPECT_DOUBLE_EQ(velocity, 0.0);
  }
  // Resuming must slew from the retained command, not re-seed from lagging feedback.
  const auto resumed = core.update(feedback, {0.0, 1.0, 0.0}, true, 0.004);
  EXPECT_GT(resumed.steering_position[0], moving.steering_position[0]);
  EXPECT_LE(
    resumed.steering_position[0] - moving.steering_position[0],
    config.setpoint.maximum_steering_slew_radps * 0.004 + 1e-12);
}

TEST_F(ControlCoreTest, ReactivationSamplesTheNewInitialPositionOnce)
{
  auto output = core.update(feedback, {0.0, 1.0, 0.0}, true, 0.004);
  core.deactivate();
  for (auto & module : feedback) {
    module.steering_position = module.steering_angle = -0.4;
  }
  ASSERT_TRUE(core.activate(feedback));
  output = core.update(feedback, {}, false, 0.004);
  for (auto target : output.steering_position) {
    EXPECT_DOUBLE_EQ(target, -0.4);
  }
  for (auto & module : feedback) {
    module.steering_position = module.steering_angle = -0.395;
  }
  output = core.update(feedback, {}, true, 0.004);
  for (auto target : output.steering_position) {
    EXPECT_DOUBLE_EQ(target, -0.4);
  }
}

TEST_F(ControlCoreTest, ModuleAtInstantaneousRotationCentreKeepsItsLastTarget)
{
  for (auto & module : feedback) {
    module.steering_position = module.steering_angle = 0.2;
  }
  ASSERT_TRUE(core.activate(feedback));
  // Module0 is at(.5,.4): vx=.04,vy=-.05,w=.1 gives near-zero module0 speed.
  // Discretization residue remains below the configured0.001m/s deadband.
  for (int cycle = 0; cycle < 30; ++cycle) {
    feedback[0].steering_position = feedback[0].steering_angle = 0.2 + (cycle % 2 ? 0.004 : -0.004);
    const auto output = core.update(feedback, {0.04, -0.05, 0.1}, true, 0.004);
    EXPECT_DOUBLE_EQ(output.steering_position[0], 0.2);
    EXPECT_DOUBLE_EQ(output.drive_velocity[0], 0.0);
    for (size_t i = 1; i < 4; ++i) {
      feedback[i].steering_position = feedback[i].steering_angle = output.steering_position[i];
    }
  }
}

TEST_F(ControlCoreTest, ExternalPushLatchesHoldFaultUntilSuccessfulReactivation)
{
  const double displaced = config.max_encoder_difference + 0.01;
  feedback[0].steering_position = feedback[0].steering_angle = displaced;
  auto output = core.update(feedback, {}, true, 0.004);
  EXPECT_EQ(output.status, ControlStatus::steering_hold_fault);
  EXPECT_TRUE(core.hold_fault_latched());
  EXPECT_DOUBLE_EQ(output.steering_position[0], 0.0);
  for (auto velocity : output.drive_velocity) {
    EXPECT_DOUBLE_EQ(velocity, 0.0);
  }
  feedback[0].steering_position = feedback[0].steering_angle = 0.0;
  output = core.update(feedback, {1.0, 0.0, 0.0}, true, 0.004);
  EXPECT_EQ(output.status, ControlStatus::steering_hold_fault);
  for (auto velocity : output.drive_velocity) {
    EXPECT_DOUBLE_EQ(velocity, 0.0);
  }
  core.deactivate();
  feedback[0].valid = false;
  EXPECT_FALSE(core.activate(feedback));
  EXPECT_TRUE(core.hold_fault_latched());
  feedback[0].valid = true;
  ASSERT_TRUE(core.activate(feedback));
  EXPECT_FALSE(core.hold_fault_latched());
  output = core.update(feedback, {1.0, 0.0, 0.0}, true, 0.004);
  EXPECT_EQ(output.status, ControlStatus::running);
  EXPECT_GT(output.drive_velocity[0], 0.0);
}

TEST_F(ControlCoreTest, HoldFaultUsesExistingToleranceAndDoesNotAcceptSlowCumulativeDrift)
{
  feedback[0].steering_position = feedback[0].steering_angle = config.max_encoder_difference;
  auto output = core.update(feedback, {}, true, 0.004);
  EXPECT_EQ(output.status, ControlStatus::running);
  feedback[0].steering_position += 0.000001;
  feedback[0].steering_angle = feedback[0].steering_position;
  output = core.update(feedback, {}, true, 0.004);
  EXPECT_EQ(output.status, ControlStatus::steering_hold_fault);
  EXPECT_DOUBLE_EQ(output.steering_position[0], 0.0);
}

TEST_F(ControlCoreTest, ADeadbandModuleDeviationStopsAllModulesAndLatches)
{
  core.update(feedback, {0.04, -0.05, 0.1}, true, 0.004);
  feedback[0].steering_position = feedback[0].steering_angle = config.max_encoder_difference + 0.01;
  const auto output = core.update(feedback, {0.04, -0.05, 0.1}, true, 0.004);
  EXPECT_EQ(output.status, ControlStatus::steering_hold_fault);
  EXPECT_DOUBLE_EQ(output.steering_position[0], 0.0);
  for (auto velocity : output.drive_velocity) {
    EXPECT_DOUBLE_EQ(velocity, 0.0);
  }
}
}  // namespace
}  // namespace swerve_driver
