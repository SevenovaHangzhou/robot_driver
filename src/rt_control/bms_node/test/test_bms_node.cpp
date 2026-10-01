#include <gtest/gtest.h>

#include "bms_node/node.hpp"

#include <chrono>
#include <memory>
#include <thread>

TEST(BmsNodeTest, MissingConfiguredCanInterfaceStopsCleanly)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).parameter_overrides({
    {"can_interface", "bms_missing"},
    {"reconnect_period_s", 0.01},
  });

  auto node = bms_node::make_node(options);
  std::this_thread::sleep_for(std::chrono::milliseconds{30});
  node.reset();
  context->shutdown("test complete");
}
