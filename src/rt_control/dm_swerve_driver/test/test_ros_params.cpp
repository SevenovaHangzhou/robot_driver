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
      rclcpp::Parameter{"odometry.max_imu_yaw_step_rad", 0.4},
      rclcpp::Parameter{"steering.rezero_tolerance_rad", 0.02},
      rclcpp::Parameter{
        "odometry.pose_covariance_diagonal",
        std::vector<double>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}},
      rclcpp::Parameter{"odometry.imu_fallback_covariance_scale", 12.0},
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
  EXPECT_DOUBLE_EQ(parameters.odometry.max_imu_yaw_step_rad, 0.4);
  EXPECT_DOUBLE_EQ(parameters.steering.rezero_tolerance_rad, 0.02);
  EXPECT_DOUBLE_EQ(parameters.odometry.pose_covariance_diagonal[5], 6.0);
  EXPECT_DOUBLE_EQ(parameters.odometry.imu_fallback_covariance_scale, 12.0);
  EXPECT_EQ(parameters.safety.auto_recovery_limit, 5U);
  EXPECT_DOUBLE_EQ(parameters.steering.zero_offset_rad[3], 0.4);
  EXPECT_EQ(parameters.motors.drive_mst_id[0], 31U);
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
