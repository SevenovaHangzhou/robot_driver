#include <gtest/gtest.h>

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
      rclcpp::Parameter{"steering.zero_offset_rad", std::vector<double>{0.1, 0.2, 0.3, 0.4}},
      rclcpp::Parameter{"motors.drive_mst_id", std::vector<std::int64_t>{31, 32, 33, 34}}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("params_overrides", options);
  declare_driver_parameters(*node);
  const auto parameters = load_driver_parameters(*node);

  EXPECT_DOUBLE_EQ(parameters.steering.max_ff_speed_radps, 0.5);
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

}  // namespace
}  // namespace dm_swerve_driver
