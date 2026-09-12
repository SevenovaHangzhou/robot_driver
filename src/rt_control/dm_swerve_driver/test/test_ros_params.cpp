#include <gtest/gtest.h>

#include <memory>
#include <limits>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "dm_swerve_driver/ros_params.hpp"

namespace dm_swerve_driver {
namespace {

class RosContextEnvironment final : public ::testing::Environment {
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      int argc{0};
      rclcpp::init(argc, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

const auto * const environment =
  ::testing::AddGlobalTestEnvironment(new RosContextEnvironment{});

TEST(RosParametersTest, DeclaresAndLoadsValidDefaults)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("params_defaults");
  EXPECT_NO_THROW(declare_driver_parameters(*node));
  const auto parameters = load_driver_parameters(*node);
  EXPECT_TRUE(parameter_errors(parameters).empty());
  EXPECT_DOUBLE_EQ(parameters.control.rate_hz, 100.0);
  EXPECT_DOUBLE_EQ(parameters.steering.max_ff_speed_radps, 3.0);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_min_rad, -kPi);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_max_rad, kPi);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_margin_rad, 0.0);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_tolerance_rad, 0.0);
  EXPECT_DOUBLE_EQ(parameters.odometry.slip_residual_threshold, 0.25);
  EXPECT_DOUBLE_EQ(parameters.odometry.slip_covariance_scale, 4.0);
}

TEST(RosParametersTest, AppliesScalarAndArrayOverrides)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter{"steering.max_ff_speed_radps", 0.5},
      rclcpp::Parameter{"can.allow_fallback_limits", true},
      rclcpp::Parameter{"can.write_timeout_us", std::int64_t{1500}},
      rclcpp::Parameter{"steering.flip_hysteresis_rad", 0.2},
      rclcpp::Parameter{"steering.max_slew_radps", 1.25},
      rclcpp::Parameter{"steering.joint_limit_min_rad", -2.8},
      rclcpp::Parameter{"steering.joint_limit_max_rad", 2.9},
      rclcpp::Parameter{"steering.joint_limit_margin_rad", 0.1},
      rclcpp::Parameter{"steering.joint_limit_tolerance_rad", 0.02},
      rclcpp::Parameter{"odometry.max_imu_yaw_step_rad", 0.4},
      rclcpp::Parameter{"steering.rezero_tolerance_rad", 0.02},
      rclcpp::Parameter{
        "odometry.pose_covariance_diagonal",
        std::vector<double>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}},
      rclcpp::Parameter{"odometry.imu_fallback_covariance_scale", 12.0},
      rclcpp::Parameter{"odometry.slip_residual_threshold", 0.2},
      rclcpp::Parameter{"odometry.slip_covariance_scale", 6.0},
      rclcpp::Parameter{"safety.auto_recovery_limit", std::int64_t{5}},
      rclcpp::Parameter{"steering.zero_offset_rad", std::vector<double>{0.1, 0.2, 0.3, 0.4}},
      rclcpp::Parameter{"motors.drive_mst_id", std::vector<std::int64_t>{31, 32, 33, 34}}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("params_overrides", options);
  declare_driver_parameters(*node);
  const auto parameters = load_driver_parameters(*node);

  EXPECT_DOUBLE_EQ(parameters.steering.max_ff_speed_radps, 0.5);
  EXPECT_TRUE(parameters.can.allow_fallback_limits);
  EXPECT_EQ(parameters.can.write_timeout_us, 1500);
  EXPECT_DOUBLE_EQ(parameters.steering.flip_hysteresis_rad, 0.2);
  EXPECT_DOUBLE_EQ(parameters.steering.max_slew_radps, 1.25);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_min_rad, -2.8);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_max_rad, 2.9);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_margin_rad, 0.1);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_tolerance_rad, 0.02);
  EXPECT_DOUBLE_EQ(parameters.odometry.max_imu_yaw_step_rad, 0.4);
  EXPECT_DOUBLE_EQ(parameters.steering.rezero_tolerance_rad, 0.02);
  EXPECT_DOUBLE_EQ(parameters.odometry.pose_covariance_diagonal[5], 6.0);
  EXPECT_DOUBLE_EQ(parameters.odometry.imu_fallback_covariance_scale, 12.0);
  EXPECT_DOUBLE_EQ(parameters.odometry.slip_residual_threshold, 0.2);
  EXPECT_DOUBLE_EQ(parameters.odometry.slip_covariance_scale, 6.0);
  EXPECT_EQ(parameters.safety.auto_recovery_limit, 5U);
  EXPECT_DOUBLE_EQ(parameters.steering.zero_offset_rad[3], 0.4);
  EXPECT_EQ(parameters.motors.drive_mst_id[0], 31U);
}

TEST(RosParametersTest, DeclaresAndLoadsStrictKincoConfiguration)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter{"kinco.ethercat.dc_cycle_ns", std::int64_t{1000000}},
      rclcpp::Parameter{"kinco.ethercat.pdo_watchdog_ms", std::int64_t{20}},
      rclcpp::Parameter{"kinco.steering_encoder_resolution",
        std::vector<std::int64_t>{10000, 10000, 10000, 10000}},
      rclcpp::Parameter{"kinco.drive_encoder_resolution",
        std::vector<std::int64_t>{10000, 10000, 10000, 10000}},
      rclcpp::Parameter{"kinco.encoder.expected_counts_per_revolution",
        std::vector<std::int64_t>{65536, 65536, 65536, 65536}},
      rclcpp::Parameter{"kinco.encoder.expected_distinguishable_revolutions",
        std::vector<std::int64_t>{24, 24, 24, 24}},
      rclcpp::Parameter{"kinco.encoder.ring_gear_teeth",
        std::vector<std::int64_t>{120, 120, 120, 120}},
      rclcpp::Parameter{"kinco.encoder.pinion_teeth",
        std::vector<std::int64_t>{20, 20, 20, 20}},
      rclcpp::Parameter{"kinco.encoder.source_disagreement_threshold_rad", 0.1},
      rclcpp::Parameter{"kinco.encoder.maximum_offline_axis_motion_rad", 0.2},
      rclcpp::Parameter{"kinco.encoder.maximum_rejoin_correction_rad", 0.01},
      rclcpp::Parameter{"kinco.encoder.snapshot_path", "/tmp/encoder.snapshot"}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("kinco_params", options);

  declare_kinco_parameters(*node);
  const auto parameters = load_kinco_parameters(*node);

  EXPECT_TRUE(kinco_parameter_errors(parameters).empty());
  EXPECT_EQ(parameters.dc_cycle_ns, 1000000);
  EXPECT_EQ(parameters.encoder_expected_counts_per_revolution[3], 65536U);
  EXPECT_EQ(parameters.encoder_ring_gear_teeth[0], 120U);
  EXPECT_EQ(parameters.encoder_snapshot_path, "/tmp/encoder.snapshot");
}

TEST(RosParametersTest, RejectsWrongSizedArrays)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter{"steering.zero_offset_rad", std::vector<double>{0.1, 0.2, 0.3}}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("params_bad_array", options);
  declare_driver_parameters(*node);
  EXPECT_THROW(
    static_cast<void>(load_driver_parameters(*node)),
    std::invalid_argument);
}

TEST(RosParametersTest, RejectsRealtimePriorityBeforeNarrowing)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter{
        "control.realtime_priority", std::int64_t{std::numeric_limits<std::int64_t>::max()}}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "params_bad_priority", options);
  declare_driver_parameters(*node);
  EXPECT_THROW(load_driver_parameters(*node), std::invalid_argument);
}

}  // namespace
}  // namespace dm_swerve_driver
