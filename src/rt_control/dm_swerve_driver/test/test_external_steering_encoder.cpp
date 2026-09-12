#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <stdexcept>

#include "dm_swerve_driver/external_steering_encoder.hpp"

namespace dm_swerve_driver {
namespace {

constexpr double kTolerance{1e-12};

[[nodiscard]] ExternalSteeringEncoderConfig encoder_config()
{
  return ExternalSteeringEncoderConfig{
    65536U,
    24U,
    120U,
    20U,
    1,
    0.0,
    0.2,
    0.2};
}

[[nodiscard]] EncoderHardwareInfo hardware_info()
{
  return EncoderHardwareInfo{65536U, 24U};
}

TEST(ExternalSteeringEncoderTest, UsesExactToothCountsDirectionAndOffset)
{
  auto config = encoder_config();
  EXPECT_NEAR(
    external_encoder_position_to_axis_angle(65536U, config), kPi / 3.0, kTolerance);

  config.direction = -1;
  config.installation_offset_rad = 0.1;
  EXPECT_NEAR(
    external_encoder_position_to_axis_angle(65536U, config),
    -kPi / 3.0 - 0.1,
    kTolerance);
}

TEST(ExternalSteeringEncoderTest, RejectsInvalidEncoderKinematics)
{
  auto config = encoder_config();
  config.pinion_teeth = 0U;
  EXPECT_THROW(validate_external_encoder_config(config), std::invalid_argument);
  config = encoder_config();
  config.direction = 0;
  EXPECT_THROW(validate_external_encoder_config(config), std::invalid_argument);
}

TEST(ExternalSteeringEncoderTest, StartupAcceptsMatchingIdentitySnapshotAndMotorSource)
{
  const auto config = encoder_config();
  const EncoderPersistentRecord persisted{65536U, hardware_info()};
  const auto result = validate_external_encoder_startup(
    config, hardware_info(), 65536U, persisted, kPi / 3.0, SteeringAngleLimits{});

  EXPECT_TRUE(result.accepted());
  EXPECT_EQ(result.failure, EncoderStartupFailure::none);
  EXPECT_NEAR(result.axis_angle_rad, kPi / 3.0, kTolerance);
}

TEST(ExternalSteeringEncoderTest, StartupRejectsMissingOrMismatchedHardwareEvidence)
{
  const auto config = encoder_config();
  const auto missing = validate_external_encoder_startup(
    config, hardware_info(), 65536U, std::nullopt, kPi / 3.0, SteeringAngleLimits{});
  EXPECT_EQ(missing.failure, EncoderStartupFailure::snapshot_missing);

  auto wrong_hardware = hardware_info();
  wrong_hardware.counts_per_revolution /= 2U;
  const EncoderPersistentRecord persisted{65536U, hardware_info()};
  const auto mismatch = validate_external_encoder_startup(
    config, wrong_hardware, 65536U, persisted, kPi / 3.0, SteeringAngleLimits{});
  EXPECT_EQ(mismatch.failure, EncoderStartupFailure::hardware_mismatch);
}

TEST(ExternalSteeringEncoderTest, StartupRejectsImplausibleOfflineJump)
{
  auto config = encoder_config();
  config.maximum_offline_axis_motion_rad = 0.1;
  const EncoderPersistentRecord persisted{0U, hardware_info()};
  const auto result = validate_external_encoder_startup(
    config, hardware_info(), 65536U, persisted, kPi / 3.0, SteeringAngleLimits{});

  EXPECT_EQ(result.failure, EncoderStartupFailure::persistent_position_jump);
}

TEST(ExternalSteeringEncoderTest, StartupUsesLinearBoundedAngleForCrossCheck)
{
  auto config = encoder_config();
  config.source_disagreement_threshold_rad = 0.05;
  const EncoderPersistentRecord persisted{65536U, hardware_info()};
  const auto disagreement = validate_external_encoder_startup(
    config, hardware_info(), 65536U, persisted, kPi / 3.0 + 0.1,
    SteeringAngleLimits{});
  EXPECT_EQ(disagreement.failure, EncoderStartupFailure::source_disagreement);

  config.maximum_offline_axis_motion_rad = 4.0;
  const EncoderPersistentRecord boundary_record{196608U, hardware_info()};
  const auto boundary = validate_external_encoder_startup(
    config, hardware_info(), 196608U, boundary_record, -kPi,
    SteeringAngleLimits{});
  EXPECT_EQ(boundary.failure, EncoderStartupFailure::source_disagreement);
}

TEST(ExternalSteeringEncoderTest, StartupRejectsAxisOutsideMechanicalTolerance)
{
  auto config = encoder_config();
  config.maximum_offline_axis_motion_rad = 5.0;
  const EncoderPersistentRecord persisted{262144U, hardware_info()};
  const auto result = validate_external_encoder_startup(
    config, hardware_info(), 262144U, persisted, 4.0 * kPi / 3.0,
    SteeringAngleLimits{});

  EXPECT_EQ(result.failure, EncoderStartupFailure::mechanical_limit);
}

TEST(ExternalSteeringEncoderTest, RuntimeFallbackAndRecoveryRemainContinuous)
{
  SteeringAngleSourceSelector selector{0.2, 0.05};
  auto selected = selector.select(
    SteeringAngleSample{1.0, true}, SteeringAngleSample{0.9, true});
  ASSERT_TRUE(selected.valid);
  EXPECT_EQ(selected.source, SteeringAngleSource::external_encoder);
  EXPECT_NEAR(selected.angle_rad, 1.0, kTolerance);

  selected = selector.select(
    SteeringAngleSample{}, SteeringAngleSample{1.0, true});
  ASSERT_TRUE(selected.valid);
  EXPECT_TRUE(selected.degraded);
  EXPECT_EQ(selected.source, SteeringAngleSource::motor_backup);
  EXPECT_NEAR(selected.angle_rad, 1.1, kTolerance);

  selected = selector.select(
    SteeringAngleSample{1.3, true}, SteeringAngleSample{1.2, true});
  ASSERT_TRUE(selected.valid);
  EXPECT_EQ(selected.source, SteeringAngleSource::external_encoder);
  EXPECT_NEAR(selected.angle_rad, 1.1, kTolerance);

  selected = selector.select(
    SteeringAngleSample{1.35, true}, SteeringAngleSample{1.25, true});
  ASSERT_TRUE(selected.valid);
  EXPECT_NEAR(selected.angle_rad, 1.2, kTolerance);

  for (const double expected : {1.25, 1.3, 1.35}) {
    selected = selector.select(
      SteeringAngleSample{1.35, true}, SteeringAngleSample{1.25, true});
    ASSERT_TRUE(selected.valid);
    EXPECT_NEAR(selected.angle_rad, expected, kTolerance);
  }
}

TEST(ExternalSteeringEncoderTest, RuntimeRejectsDisagreementAndUnalignedBackup)
{
  SteeringAngleSourceSelector selector{0.1, 0.05};
  auto selected = selector.select(
    SteeringAngleSample{1.0, true}, SteeringAngleSample{0.0, true});
  EXPECT_FALSE(selected.valid);
  EXPECT_TRUE(selected.disagreement);

  selector.reset();
  selected = selector.select(
    SteeringAngleSample{}, SteeringAngleSample{0.5, true});
  EXPECT_FALSE(selected.valid);
  EXPECT_EQ(selected.source, SteeringAngleSource::unavailable);
}

}  // namespace
}  // namespace dm_swerve_driver
