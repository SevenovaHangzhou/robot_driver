#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <net/if.h>
#include <stdexcept>
#include <vector>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "lpms_nav3_can/node.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "sensor_msgs/msg/imu.hpp"

TEST(LpmsNodeTest, MissingInterfaceReportsDiagnosticsWithoutFabricatingSamples)
{
  ASSERT_EQ(if_nametoindex("lpms_absent"), 0U);
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).use_intra_process_comms(true);
  options.parameter_overrides({
    {"node_id", 1}, {"can_interface", "lpms_absent"}, {"frame_id", "imu_link"}});
  auto driver = lpms_nav3_can::make_node(options);
  auto collector = std::make_shared<rclcpp::Node>("lpms_collector", options);
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostics;
  std::size_t samples{0U};
  auto diagnostic_subscription = collector->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/lpms_nav3_can_node/diagnostics", robot_interfaces_qos::diagnostic(),
    [&](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {diagnostics = message;});
  auto imu_subscription = collector->create_subscription<sensor_msgs::msg::Imu>(
    "/lpms_nav3_can_node/data", robot_interfaces_qos::fast_state(),
    [&](sensor_msgs::msg::Imu::SharedPtr) {++samples;});
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor{executor_options};
  executor.add_node(driver);
  executor.add_node(collector);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{3};
  while (!diagnostics && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
  }
  ASSERT_TRUE(diagnostics);
  ASSERT_EQ(diagnostics->status.size(), 1U);
  const auto & status = diagnostics->status.front();
  EXPECT_EQ(status.level, status.ERROR);
  EXPECT_EQ(status.message, "SocketCAN interface unavailable");
  EXPECT_EQ(samples, 0U);
  for (const auto & value : status.values) {
    if (value.key == "socket_connected") {EXPECT_EQ(value.value, "false");}
    if (value.key == "published_samples") {EXPECT_EQ(value.value, "0");}
  }
  context->shutdown("test complete");
}

class LpmsCovarianceParameterTest : public testing::Test
{
protected:
  void SetUp() override
  {
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    options.context(context);
    parameters = {
      {"node_id", 1}, {"can_interface", "lpms_absent"}, {"frame_id", "imu_link"}};
  }

  void TearDown() override {context->shutdown("test complete");}

  rclcpp::Context::SharedPtr context;
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> parameters;
};

TEST_F(LpmsCovarianceParameterTest, DefaultsAreNineZerosAndReadOnly)
{
  options.parameter_overrides(parameters);
  const auto node = lpms_nav3_can::make_node(options);
  for (const auto * name : {"orientation_covariance", "angular_velocity_covariance"}) {
    EXPECT_EQ(node->get_parameter(name).as_double_array(), std::vector<double>(9U, 0.0));
    EXPECT_TRUE(node->describe_parameter(name).read_only);
  }
}

TEST_F(LpmsCovarianceParameterTest, AcceptsConfiguredMatrices)
{
  const std::vector<double> orientation{0.01, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.03};
  const std::vector<double> angular_velocity{0.04, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.06};
  parameters.emplace_back("orientation_covariance", orientation);
  parameters.emplace_back("angular_velocity_covariance", angular_velocity);
  options.parameter_overrides(parameters);
  const auto node = lpms_nav3_can::make_node(options);
  EXPECT_EQ(node->get_parameter("orientation_covariance").as_double_array(), orientation);
  EXPECT_EQ(node->get_parameter("angular_velocity_covariance").as_double_array(), angular_velocity);
}

TEST_F(LpmsCovarianceParameterTest, RejectsWrongLengthsAndNonFiniteValues)
{
  std::vector<std::vector<double>> invalid_values{
    std::vector<double>(8U, 0.0), std::vector<double>(10U, 0.0)};
  auto non_finite = std::vector<double>(9U, 0.0);
  non_finite[4] = std::numeric_limits<double>::quiet_NaN();
  invalid_values.push_back(non_finite);
  non_finite[4] = std::numeric_limits<double>::infinity();
  invalid_values.push_back(non_finite);
  for (const auto * name : {"orientation_covariance", "angular_velocity_covariance"}) {
    for (const auto & values : invalid_values) {
      auto overrides = parameters;
      overrides.emplace_back(name, values);
      options.parameter_overrides(overrides);
      EXPECT_THROW(lpms_nav3_can::make_node(options), std::invalid_argument);
    }
  }
}
