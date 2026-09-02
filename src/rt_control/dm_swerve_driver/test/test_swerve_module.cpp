#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "dm_swerve_driver/dm_frame_codec.hpp"
#include "dm_swerve_driver/swerve_module.hpp"

namespace dm_swerve_driver {
namespace {

constexpr MotorLimits kSteeringLimits{20.0, 30.0, 10.0};
constexpr MotorLimits kDriveLimits{50.0, 100.0, 20.0};

[[nodiscard]] SteeringModuleConfig steering_config(bool inverted = false)
{
  return SteeringModuleConfig{2.0, inverted, 0.1, 30.0, 1.0, 0.9, 0.9, 0.95};
}

[[nodiscard]] DriveModuleConfig drive_config(double maximum_acceleration = 100.0)
{
  return DriveModuleConfig{5.0, true, 0.1, 2.0, 1.0, 2.0, 0.5, maximum_acceleration};
}

[[nodiscard]] SwerveModule make_module(
  SteeringModuleConfig steering = steering_config(),
  DriveModuleConfig drive = drive_config(),
  MotorLimits steering_limits = kSteeringLimits)
{
  return SwerveModule{
    DmMotorConfig{1U, 0x11U, steering_limits},
    DmMotorConfig{2U, 0x12U, kDriveLimits},
    steering,
    drive};
}

TEST(SwerveModuleTest, ConvertsMotorFeedbackToModuleUnits)
{
  auto module = make_module();
  module.steering_motor().seed_position(0.6);
  const auto drive_feedback = encode_motor_feedback(
    0x12U,
    MotorFeedback{2U, MotorError::enabled, 10.0, -5.0, 0.0, 35U, 30U},
    kDriveLimits);
  static_cast<void>(module.drive_motor().accept_feedback(drive_feedback));

  EXPECT_NEAR(module.steering_angle_rad(), 0.2, 1e-12);
  EXPECT_NEAR(module.wheel_distance_m(), -0.2, 1e-4);
  EXPECT_NEAR(module.wheel_velocity_mps(), 0.1, 2e-3);
}

TEST(SwerveModuleTest, MapsSteeringFeedforwardAndDriveUnits)
{
  auto module = make_module();
  const auto first = module.make_command(
    OptimizedModuleState{2.0, 0.3, 0.0}, 0.1);

  EXPECT_NEAR(first.steering.position, 0.8, 1e-12);
  EXPECT_DOUBLE_EQ(first.steering.velocity, 0.0);
  EXPECT_DOUBLE_EQ(first.steering.kp, 30.0);
  EXPECT_DOUBLE_EQ(first.steering.kd, 1.0);
  EXPECT_NEAR(first.drive.velocity, -100.0, 1e-12);
  EXPECT_DOUBLE_EQ(first.drive.kp, 0.0);
  EXPECT_DOUBLE_EQ(first.drive.kd, 2.0);
  EXPECT_NEAR(first.drive.torque, -3.0, 1e-12);

  const auto second = module.make_command(
    OptimizedModuleState{1.5, 0.4, 0.0}, 0.1);
  EXPECT_NEAR(second.steering.velocity, 1.8, 1e-12);
  EXPECT_NEAR(second.drive.velocity, -75.0, 1e-12);
  EXPECT_NEAR(second.drive.torque, -0.3, 1e-12);
}

TEST(SwerveModuleTest, MotorSideTorqueDividesByGearRatioAndAppliesDirection)
{
  auto config = drive_config();
  config.ks = 0.0;
  config.kv = 10.0;
  config.ka = 0.0;
  auto module = make_module(steering_config(), config);
  const auto command = module.make_command(
    OptimizedModuleState{1.0, 0.0, 0.0}, 1.0);

  // 10 Nm at the wheel becomes 2 Nm at the motor, then follows the inverted axis sign.
  EXPECT_NEAR(command.drive.torque, -2.0, 1e-12);
}

TEST(SwerveModuleTest, SteeringInversionAffectsPositionAndVelocity)
{
  auto module = make_module(steering_config(true));
  static_cast<void>(module.make_command(
      OptimizedModuleState{0.0, 0.2, 0.0}, 0.1));
  const auto command = module.make_command(
    OptimizedModuleState{0.0, 0.3, 0.0}, 0.1);

  EXPECT_NEAR(command.steering.position, -0.8, 1e-12);
  EXPECT_NEAR(command.steering.velocity, -1.8, 1e-12);
}

TEST(SwerveModuleTest, RecentersNearPmaxWithOneEquivalentPiFlip)
{
  auto config = steering_config();
  config.gear_ratio = 1.0;
  config.zero_offset_rad = 0.0;
  auto module = make_module(config);
  static_cast<void>(module.make_command(
      OptimizedModuleState{1.0, 17.9, 0.0}, 0.1));
  const auto command = module.make_command(
    OptimizedModuleState{1.0, 18.2, 0.0}, 0.1);

  EXPECT_TRUE(command.recentered);
  EXPECT_NEAR(command.continuous_angle_rad, 18.2 - kPi, 1e-12);
  EXPECT_NEAR(command.steering.position, 18.2 - kPi, 1e-12);
  EXPECT_DOUBLE_EQ(command.steering.velocity, 0.0);
  EXPECT_DOUBLE_EQ(command.wheel_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(command.drive.velocity, 0.0);

  const auto aligned = module.make_command(
    OptimizedModuleState{-1.0, 18.2 - kPi, 0.0}, 0.1);
  EXPECT_FALSE(aligned.recentered);
  EXPECT_NEAR(aligned.wheel_speed_mps, -1.0, 1e-12);
}

TEST(SwerveModuleTest, SteeringFeedforwardClampsAxisSideStepBeforeGearRatio)
{
  auto config = steering_config();
  config.max_ff_speed_radps = 0.25;
  auto module = make_module(config);
  static_cast<void>(module.make_command(
      OptimizedModuleState{0.0, 0.0, 0.0}, 0.1));
  const auto command = module.make_command(
    OptimizedModuleState{0.0, 1.0, 0.0}, 0.1);

  EXPECT_NEAR(command.steering.velocity, 0.9 * 0.25 * 2.0, 1e-12);
}

TEST(SwerveModuleTest, SteeringPositionIsClampedInsideMappingBoundary)
{
  auto config = steering_config();
  config.gear_ratio = 1.0;
  config.zero_offset_rad = 0.0;
  auto module = make_module(config);
  const auto command = module.make_command(
    OptimizedModuleState{1.0, 100.0, 0.0}, 0.1);
  EXPECT_NEAR(std::abs(command.steering.position), 0.95 * kSteeringLimits.position_max, 1e-12);
}

TEST(SwerveModuleTest, AccelerationLimitFeedsKaAndHardGateBypassesRamp)
{
  auto module = make_module(steering_config(), drive_config(2.0));
  const auto limited = module.make_command(
    OptimizedModuleState{10.0, 0.0, 0.0}, 0.1);
  EXPECT_NEAR(limited.wheel_speed_mps, 0.2, 1e-12);
  EXPECT_NEAR(limited.drive.velocity, -10.0, 1e-12);

  const auto gated = module.make_command(
    OptimizedModuleState{10.0, 0.0, 0.0}, 0.1, true);
  EXPECT_DOUBLE_EQ(gated.wheel_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(gated.drive.velocity, 0.0);
  EXPECT_DOUBLE_EQ(gated.drive.torque, 0.0);
}

TEST(SwerveModuleTest, EncodesSteeringThenDriveFrames)
{
  auto module = make_module();
  const auto command = module.make_command(
    OptimizedModuleState{0.5, 0.2, 0.0}, 0.1);
  const auto frames = module.encode_command_frames(command);
  EXPECT_EQ(frames[0].id, 1U);
  EXPECT_EQ(frames[1].id, 2U);

  const auto steering = decode_mit_command(frames[0], kSteeringLimits);
  const auto drive = decode_mit_command(frames[1], kDriveLimits);
  EXPECT_NEAR(steering.position, command.steering.position, 1e-3);
  EXPECT_NEAR(drive.velocity, command.drive.velocity, 0.05);
}

TEST(SwerveModuleTest, ResetHistoryRemovesFeedforwardStep)
{
  auto module = make_module();
  static_cast<void>(module.make_command(
      OptimizedModuleState{1.0, 0.0, 0.0}, 0.1));
  static_cast<void>(module.make_command(
      OptimizedModuleState{1.0, 0.2, 0.0}, 0.1));
  module.reset_command_history();
  const auto reset = module.make_command(
    OptimizedModuleState{0.0, 0.4, 0.0}, 0.1);
  EXPECT_DOUBLE_EQ(reset.steering.velocity, 0.0);
}

TEST(SwerveModuleTest, RejectsInvalidConfigurationAndPeriod)
{
  auto invalid_steering = steering_config();
  invalid_steering.gear_ratio = 0.0;
  EXPECT_THROW(static_cast<void>(make_module(invalid_steering)), std::invalid_argument);

  auto invalid_drive = drive_config();
  invalid_drive.wheel_radius_m = 0.0;
  EXPECT_THROW(
    static_cast<void>(make_module(steering_config(), invalid_drive)), std::invalid_argument);

  auto module = make_module();
  EXPECT_THROW(
    static_cast<void>(module.make_command(OptimizedModuleState{}, 0.0)),
    std::invalid_argument);

  auto uninitialized = make_module();
  EXPECT_THROW(static_cast<void>(uninitialized.steering_angle_rad()), std::logic_error);
  EXPECT_THROW(static_cast<void>(uninitialized.wheel_distance_m()), std::logic_error);
}

}  // namespace
}  // namespace dm_swerve_driver
