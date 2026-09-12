#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include "dm_swerve_driver/kinco_params.hpp"

namespace dm_swerve_driver {
namespace {

[[nodiscard]] bool contains_error(
  const std::vector<std::string> & errors, const std::string & fragment)
{
  return std::any_of(errors.begin(), errors.end(), [&](const std::string & error) {
      return error.find(fragment) != std::string::npos;
    });
}

[[nodiscard]] KincoParameters valid_kinco_parameters()
{
  KincoParameters parameters;
  parameters.dc_cycle_ns = 1000000;
  parameters.pdo_watchdog_ms = 20;
  parameters.steering_encoder_resolution.fill(10000U);
  parameters.drive_encoder_resolution.fill(10000U);
  parameters.encoder_expected_counts_per_revolution.fill(65536U);
  parameters.encoder_expected_distinguishable_revolutions.fill(24U);
  parameters.encoder_ring_gear_teeth.fill(120U);
  parameters.encoder_pinion_teeth.fill(20U);
  parameters.encoder_source_disagreement_threshold_rad = 0.1;
  parameters.encoder_maximum_offline_axis_motion_rad = 0.2;
  parameters.encoder_maximum_rejoin_correction_rad = 0.01;
  parameters.encoder_snapshot_path = "/var/lib/robot/dm_swerve_encoder.snapshot";
  return parameters;
}

TEST(KincoParametersTest, DefaultsFailClosedUntilHardwareFactsAreFilled)
{
  const auto errors = kinco_parameter_errors(KincoParameters{});
  EXPECT_TRUE(contains_error(errors, "dc_cycle_ns"));
  EXPECT_TRUE(contains_error(errors, "pdo_watchdog_ms"));
  EXPECT_TRUE(contains_error(errors, "steering_encoder_resolution"));
  EXPECT_TRUE(contains_error(errors, "encoder_expected_counts_per_revolution"));
  EXPECT_TRUE(contains_error(errors, "encoder_ring_gear_teeth"));
  EXPECT_TRUE(contains_error(errors, "encoder_snapshot_path"));
}

TEST(KincoParametersTest, CompleteHardwareConfigurationPassesStrictValidation)
{
  const auto parameters = valid_kinco_parameters();
  EXPECT_TRUE(kinco_parameter_errors(parameters).empty());
  EXPECT_NO_THROW(validate_kinco_parameters(parameters));
}

TEST(KincoParametersTest, RejectsUnsafeTimingNodesAndSourceThresholds)
{
  auto parameters = valid_kinco_parameters();
  parameters.pdo_watchdog_ms = 1;
  parameters.encoder_node_ids[2] = parameters.encoder_node_ids[1];
  parameters.encoder_direction[0] = 0;
  parameters.encoder_source_disagreement_threshold_rad = 0.0;
  parameters.encoder_maximum_rejoin_correction_rad = 0.0;
  const auto errors = kinco_parameter_errors(parameters);

  EXPECT_TRUE(contains_error(errors, "pdo_watchdog_ms"));
  EXPECT_TRUE(contains_error(errors, "encoder_node_ids"));
  EXPECT_TRUE(contains_error(errors, "encoder_direction"));
  EXPECT_TRUE(contains_error(errors, "encoder.source_disagreement_threshold_rad"));
  EXPECT_TRUE(contains_error(errors, "encoder.maximum_rejoin_correction_rad"));
}

TEST(KincoParametersTest, BuildsBackendAndEncoderConfigurations)
{
  const auto parameters = valid_kinco_parameters();
  auto common = default_parameters();
  common.steering.gear_ratio = 2.0;
  common.drive.gear_ratio = 3.0;
  common.chassis.wheel_radius_m = 0.08;
  common.steering.zero_offset_rad[2] = 0.3;
  common.steering.inverted[2] = true;
  common.drive.inverted[2] = true;

  const auto hardware = kinco_hardware_config(common, parameters);
  const auto encoder = external_encoder_config(parameters, 2U);
  const auto canopen = encoder_canopen_config(parameters);

  EXPECT_DOUBLE_EQ(hardware.steering_gear_ratio, 2.0);
  EXPECT_DOUBLE_EQ(hardware.drive_gear_ratio, 3.0);
  EXPECT_DOUBLE_EQ(hardware.wheel_radius_m, 0.08);
  EXPECT_DOUBLE_EQ(hardware.steering_zero_offset_rad[2], 0.3);
  EXPECT_TRUE(hardware.steering_inverted[2]);
  EXPECT_TRUE(hardware.drive_inverted[2]);
  EXPECT_EQ(encoder.expected_counts_per_revolution, 65536U);
  EXPECT_EQ(encoder.ring_gear_teeth, 120U);
  EXPECT_EQ(encoder.pinion_teeth, 20U);
  EXPECT_DOUBLE_EQ(encoder.source_disagreement_threshold_rad, 0.1);
  EXPECT_EQ(canopen.node_ids[2], 3U);
  EXPECT_EQ(canopen.startup_deadline_us, parameters.encoder_feedback_deadline_us);
}

TEST(KincoParametersTest, OptionalPreopWritesRemainOptIn)
{
  auto parameters = valid_kinco_parameters();
  auto hardware = kinco_hardware_config(default_parameters(), parameters);
  EXPECT_FALSE(hardware.position_velocity_feedforward_raw.has_value());
  EXPECT_FALSE(hardware.position_acceleration_feedforward.has_value());
  EXPECT_FALSE(hardware.fault_reaction_option_code.has_value());

  parameters.write_position_feedforward = true;
  parameters.position_velocity_feedforward_raw = 80U;
  parameters.position_acceleration_feedforward = 1000U;
  parameters.write_fault_reaction_option = true;
  parameters.fault_reaction_option_code = 1;
  hardware = kinco_hardware_config(default_parameters(), parameters);
  EXPECT_EQ(hardware.position_velocity_feedforward_raw, 80U);
  EXPECT_EQ(hardware.position_acceleration_feedforward, 1000U);
  EXPECT_EQ(hardware.fault_reaction_option_code, 1);
}

}  // namespace
}  // namespace dm_swerve_driver
