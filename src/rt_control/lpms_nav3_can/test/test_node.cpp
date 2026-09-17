#include <gtest/gtest.h>

#include <chrono>
#include <net/if.h>
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
