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
TEST(BmsNodeTest, RefusesPartialOrEqualMosMappings)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).parameter_overrides({{"mos_closed_raw", 1}});
  EXPECT_THROW(static_cast<void>(bms_node::make_node(options)), std::invalid_argument);
  options.parameter_overrides({{"mos_closed_raw", 1}, {"mos_open_raw", 1}});
  EXPECT_THROW(static_cast<void>(bms_node::make_node(options)), std::invalid_argument);
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
