#include <gtest/gtest.h>

#include "bms_node/node.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "std_msgs/msg/bool.hpp"

#include <array>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

namespace
{
class BmsControlNodeTest : public testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    options_.context(context_);
    parameters_ = {
      {"can_interface", "bms_missing"}, {"automatic_discharge_control", true},
      {"request_period_s", 0.05}, {"reconnect_period_s", 2.0},
      {"max_join_voltage_delta_v", 0.5}, {"max_trip_voltage_delta_v", 1.0},
      {"max_join_current_a", 5.0}, {"min_cell_voltage_v", 3.0},
      {"max_cell_voltage_v", 3.7}, {"min_cell_temperature_c", 0.0},
      {"max_cell_temperature_c", 45.0}, {"join_stable_s", 1.0},
      {"secondary_observation_s", 1.0}, {"control_status_timeout_s", 1.0},
      {"command_timeout_s", 1.0}, {"discharge_mos_on_raw", 1},
      {"discharge_mos_off_raw", 0},
      {"relay_command_topic", "/test/bms/relay_command"},
      {"relay_feedback_topic", "/test/bms/relay_feedback"},
      {"loads_stop_request_topic", "/test/bms/stop_request"},
      {"loads_stopped_feedback_topic", "/test/bms/stopped"},
    };
  }

  void TearDown() override {context_->shutdown("test complete");}

  void topic(const std::string & name, const std::string & value)
  {
    for (auto & parameter : parameters_) {
      if (parameter.get_name() == name) {
        parameter = rclcpp::Parameter{name, value};
        return;
      }
    }
    FAIL() << "Unknown test parameter " << name;
  }

  rclcpp::Context::SharedPtr context_;
  rclcpp::NodeOptions options_;
  std::vector<rclcpp::Parameter> parameters_;
};

TEST_F(BmsControlNodeTest, RejectsEveryPairOfCollidingControlTopics)
{
  const std::array<std::string, 4U> names{
    "relay_command_topic", "relay_feedback_topic",
    "loads_stop_request_topic", "loads_stopped_feedback_topic"};
  const auto original = parameters_;
  for (std::size_t i = 0U; i < names.size(); ++i) {
    for (std::size_t j = i + 1U; j < names.size(); ++j) {
      parameters_ = original;
      topic(names[i], "/test/bms/collision");
      topic(names[j], "/test/bms/collision");
      options_.parameter_overrides(parameters_);
      EXPECT_THROW(static_cast<void>(bms_node::make_node(options_)), std::invalid_argument);
    }
  }
}

TEST_F(BmsControlNodeTest, RejectsRelativeAndAbsoluteAliases)
{
  topic("relay_command_topic", "test/bms/alias");
  topic("loads_stopped_feedback_topic", "/test/bms/alias");
  options_.parameter_overrides(parameters_);
  EXPECT_THROW(static_cast<void>(bms_node::make_node(options_)), std::invalid_argument);
}

TEST_F(BmsControlNodeTest, RejectsRemappedCrossWiring)
{
  options_.parameter_overrides(parameters_).arguments({
    "--ros-args", "-r", "/test/bms/stopped:=/test/bms/relay_command"});
  EXPECT_THROW(static_cast<void>(bms_node::make_node(options_)), std::invalid_argument);
}

TEST_F(BmsControlNodeTest, OfflineControlKeepsStopAndRelayHoldAtControlCadence)
{
  rclcpp::NodeOptions observer_options;
  observer_options.context(context_);
  auto observer = std::make_shared<rclcpp::Node>("bms_control_test", observer_options);
  std::size_t stop_count = 0U;
  std::size_t hold_count = 0U;
  std::size_t transition_logs = 0U;
  auto stop = observer->create_subscription<std_msgs::msg::Bool>(
    "/test/bms/stop_request", 10, [&](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) {++stop_count;}
    });
  auto relay = observer->create_subscription<std_msgs::msg::Bool>(
    "/test/bms/relay_command", 10, [&](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) {++hold_count;}
    });
  auto log = observer->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::QoS{1000}.reliable().transient_local(),
    [&](const rcl_interfaces::msg::Log::SharedPtr msg) {
      if (msg->name == "bms_node" && msg->msg.find(
          "waiting_for_relay_open -> stopping_loads") != std::string::npos)
      {
        ++transition_logs;
      }
    });
  auto relay_feedback = observer->create_publisher<std_msgs::msg::Bool>(
    "/test/bms/relay_feedback", 10);
  auto load_feedback = observer->create_publisher<std_msgs::msg::Bool>(
    "/test/bms/stopped", 10);
  options_.parameter_overrides(parameters_);
  auto node = bms_node::make_node(options_);
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  rclcpp::executors::SingleThreadedExecutor executor{executor_options};
  executor.add_node(observer);
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{900};
  while (std::chrono::steady_clock::now() < deadline) {
    std_msgs::msg::Bool feedback;
    feedback.data = true;
    relay_feedback->publish(feedback);
    feedback.data = false;
    load_feedback->publish(feedback);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  EXPECT_GE(stop_count, 3U);
  EXPECT_GE(hold_count, 3U);
  EXPECT_EQ(transition_logs, 1U);
  executor.remove_node(node);
  // Shutdown must also wake an offline reconnect wait immediately.
  const auto shutdown_start = std::chrono::steady_clock::now();
  node.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - shutdown_start, std::chrono::seconds{1});
  static_cast<void>(stop);
  static_cast<void>(relay);
  static_cast<void>(log);
}
}  // namespace
