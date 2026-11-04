#include <gtest/gtest.h>

#include "periph_bms/node.hpp"
#include "robot_interfaces_qos/profiles.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
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
TEST(BmsNodeTest, RefusesPartialOrEqualMosMappings)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).parameter_overrides({{"mos_closed_raw", 1}});
  EXPECT_THROW(static_cast<void>(periph_bms::make_node(options)), std::invalid_argument);
  options.parameter_overrides({{"mos_closed_raw", 1}, {"mos_open_raw", 1}});
  EXPECT_THROW(static_cast<void>(periph_bms::make_node(options)), std::invalid_argument);
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
  for (const auto * topic :
       {"/battery_state", "/battery_state/secondary", "/bms/primary/status_chinese",
        "/bms/secondary/status_chinese", "/bms/primary/fault_chinese",
        "/bms/secondary/fault_chinese"}) {
    const auto publishers = node->get_publishers_info_by_topic(topic);
    ASSERT_EQ(publishers.size(), 1U) << topic;
    EXPECT_EQ(
      publishers[0].topic_type(), std::string(topic).find("/bms/") == 0
                                    ? "std_msgs/msg/String"
                                    : "sensor_msgs/msg/BatteryState")
      << topic;
    EXPECT_EQ(publishers[0].qos_profile().reliability(), expected.reliability()) << topic;
    EXPECT_EQ(publishers[0].qos_profile().durability(), expected.durability()) << topic;
  }
  // This collector exposes no automatic discharge control; relays belong to the PLC.
  EXPECT_FALSE(node->has_parameter("automatic_discharge_control"));
  node.reset();
  context->shutdown("test complete");
}
