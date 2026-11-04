#include <gtest/gtest.h>

#include "periph_bms/node.hpp"
#include "robot_interfaces_qos/profiles.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>

TEST(BmsNodeTest, MissingConfiguredCanInterfaceStopsCleanly)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).parameter_overrides({
    {"transport", "socketcan"},
    {"can_interface", "bms_missing"},
    {"expected_adapter_serial", "test_serial"},
    {"reconnect_period_s", 0.01},
  });

  auto node = periph_bms::make_node(options);
  std::this_thread::sleep_for(std::chrono::milliseconds{30});
  node.reset();
  context->shutdown("test complete");
}

TEST(BmsNodeTest, RefusesAutomaticControlWithoutCommissioningParameters)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).parameter_overrides({
    {"automatic_discharge_control", true},
    {"expected_adapter_serial", "test_serial"},
  });
  EXPECT_THROW(static_cast<void>(periph_bms::make_node(options)), std::invalid_argument);
  context->shutdown("test complete");
}

TEST(BmsNodeTest, ConfiguredControlStillWaitsForCanAndFeedback)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).parameter_overrides({
    {"can_interface", "bms_missing"},
    {"expected_adapter_serial", "test_serial"},
    {"reconnect_period_s", 0.01},
    {"automatic_discharge_control", true},
    {"max_join_voltage_delta_v", 0.5},
    {"max_trip_voltage_delta_v", 1.0},
    {"max_join_current_a", 5.0},
    {"min_cell_voltage_v", 3.0},
    {"max_cell_voltage_v", 3.7},
    {"min_cell_temperature_c", 0.0},
    {"max_cell_temperature_c", 45.0},
    {"join_stable_s", 1.0},
    {"secondary_observation_s", 1.0},
    {"control_status_timeout_s", 1.0},
    {"command_timeout_s", 1.0},
    {"discharge_mos_on_raw", 1},
    {"discharge_mos_off_raw", 0},
    {"relay_command_topic", "/test/k2/command"},
    {"relay_feedback_topic", "/test/k2/closed"},
    {"loads_stop_request_topic", "/test/loads/stop"},
    {"loads_stopped_feedback_topic", "/test/loads/stopped"},
  });
  auto node = periph_bms::make_node(options);
  std::this_thread::sleep_for(std::chrono::milliseconds{30});
  node.reset();
  context->shutdown("test complete");
}

TEST(BmsNodeTest, RefusesConflictingBatteryAddresses)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).parameter_overrides({
    {"primary_bms_address", 1},
    {"secondary_bms_address", 1},
    {"expected_adapter_serial", "test_serial"},
  });
  EXPECT_THROW(static_cast<void>(periph_bms::make_node(options)), std::invalid_argument);
  context->shutdown("test complete");
}

TEST(BmsNodeTest, RefusesInvalidInterfaceName)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).parameter_overrides({
    {"can_interface", "../can0"},
    {"expected_adapter_serial", "test_serial"},
  });
  EXPECT_THROW(static_cast<void>(periph_bms::make_node(options)), std::invalid_argument);
  context->shutdown("test complete");
}

TEST(BmsNodeTest, AllowsCanCardWithoutUsbSerial)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).parameter_overrides({
    {"can_interface", "bms_missing"},
    {"expected_adapter_serial", ""},
    {"reconnect_period_s", 0.01},
  });
  auto node = periph_bms::make_node(options);
  std::this_thread::sleep_for(std::chrono::milliseconds{30});
  node.reset();
  context->shutdown("test complete");
}

TEST(BmsNodeTest, PublishesBothBatteryStatesWithTheContractStateQos)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).parameter_overrides({
    {"can_interface", "bms_missing"},
    {"expected_adapter_serial", "test_serial"},
    {"reconnect_period_s", 0.01},
  });
  auto node = periph_bms::make_node(options);
  const auto expected = robot_interfaces_qos::state();
  for (const auto * topic : {"/battery_state", "/battery_state/secondary"}) {
    const auto publishers = node->get_publishers_info_by_topic(topic);
    ASSERT_EQ(publishers.size(), 1U) << topic;
    EXPECT_EQ(publishers[0].topic_type(), "sensor_msgs/msg/BatteryState") << topic;
    EXPECT_EQ(publishers[0].qos_profile().reliability(), expected.reliability()) << topic;
    EXPECT_EQ(publishers[0].qos_profile().durability(), expected.durability()) << topic;
  }
  // Discharge control is opt-in: without commissioning it never commands the relay.
  EXPECT_FALSE(node->get_parameter("automatic_discharge_control").as_bool());
  node.reset();
  context->shutdown("test complete");
}
