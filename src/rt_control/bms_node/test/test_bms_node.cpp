#include <gtest/gtest.h>

#include "bms_node/node.hpp"

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

  auto node = bms_node::make_node(options);
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
  EXPECT_THROW(static_cast<void>(bms_node::make_node(options)), std::invalid_argument);
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
  auto node = bms_node::make_node(options);
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
  EXPECT_THROW(static_cast<void>(bms_node::make_node(options)), std::invalid_argument);
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
  EXPECT_THROW(static_cast<void>(bms_node::make_node(options)), std::invalid_argument);
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
  auto node = bms_node::make_node(options);
  std::this_thread::sleep_for(std::chrono::milliseconds{30});
  node.reset();
  context->shutdown("test complete");
}
