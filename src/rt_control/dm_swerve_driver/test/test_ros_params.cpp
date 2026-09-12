#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <memory>
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

TEST(RosParametersTest, DeclaresAndLoadsCommonDefaults)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("params_defaults");

  declare_driver_parameters(*node);
  const auto parameters = load_driver_parameters(*node);

  EXPECT_TRUE(parameter_errors(parameters).empty());
  EXPECT_DOUBLE_EQ(parameters.control.rate_hz, 100.0);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_min_rad, -kPi);
  EXPECT_DOUBLE_EQ(parameters.odometry.slip_residual_threshold, 0.25);
}

TEST(RosParametersTest, AppliesCommonScalarAndArrayOverrides)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter{"steering.max_slew_radps", 1.25},
      rclcpp::Parameter{"steering.joint_limit_min_rad", -2.8},
      rclcpp::Parameter{"steering.joint_limit_max_rad", 2.9},
      rclcpp::Parameter{"steering.joint_limit_margin_rad", 0.1},
      rclcpp::Parameter{"steering.zero_offset_rad",
        std::vector<double>{0.1, 0.2, 0.3, 0.4}},
      rclcpp::Parameter{"drive.invert", std::vector<bool>{true, false, true, false}},
      rclcpp::Parameter{"safety.auto_recovery_limit", std::int64_t{5}}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("params_overrides", options);

  declare_driver_parameters(*node);
  const auto parameters = load_driver_parameters(*node);

  EXPECT_DOUBLE_EQ(parameters.steering.max_slew_radps, 1.25);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_min_rad, -2.8);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_max_rad, 2.9);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_margin_rad, 0.1);
  EXPECT_DOUBLE_EQ(parameters.steering.zero_offset_rad[3], 0.4);
  EXPECT_TRUE(parameters.drive.inverted[0]);
  EXPECT_EQ(parameters.safety.auto_recovery_limit, 5U);
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
}

TEST(RosParametersTest, RejectsInvalidArrayAndPriorityBeforeNarrowing)
{
  rclcpp::NodeOptions bad_array;
  bad_array.parameter_overrides({
      rclcpp::Parameter{"steering.zero_offset_rad", std::vector<double>{0.1, 0.2, 0.3}}});
  auto array_node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("params_bad_array", bad_array);
  declare_driver_parameters(*array_node);
  EXPECT_THROW(load_driver_parameters(*array_node), std::invalid_argument);

  rclcpp::NodeOptions bad_priority;
  bad_priority.parameter_overrides({
      rclcpp::Parameter{"control.realtime_priority",
        std::numeric_limits<std::int64_t>::max()}});
  auto priority_node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("params_bad_priority", bad_priority);
  declare_driver_parameters(*priority_node);
  EXPECT_THROW(load_driver_parameters(*priority_node), std::invalid_argument);
}

}  // namespace
}  // namespace dm_swerve_driver
