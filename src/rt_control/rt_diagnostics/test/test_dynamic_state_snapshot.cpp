#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "rt_diagnostics/dynamic_state_snapshot.hpp"

namespace rt_diagnostics
{
namespace
{

control_msgs::msg::DynamicJointState makeSnapshot()
{
  control_msgs::msg::DynamicJointState message;
  message.joint_names = {"axis_a", "axis_b"};
  message.interface_values.resize(2U);
  message.interface_values[0].interface_names = {"position", "status_word"};
  message.interface_values[0].values = {1.0, 0x0027};
  message.interface_values[1].interface_names = {"status_word"};
  message.interface_values[1].values = {0x0040};
  return message;
}

TEST(DynamicStateSnapshotTest, ReplacesTheWholeSnapshotInsteadOfRetainingOldInterfaces)
{
  std::unordered_map<std::string, double> values{{"stale_axis/status_word", 0x0027}};
  const auto message = makeSnapshot();

  ASSERT_TRUE(replaceDynamicStateSnapshot(message, values));

  EXPECT_EQ(values.count("stale_axis/status_word"), 0U);
  EXPECT_EQ(values.at("axis_a/status_word"), 0x0027);
  EXPECT_EQ(values.at("axis_b/status_word"), 0x0040);
}

TEST(DynamicStateSnapshotTest, AValidPartialGraphStillDropsPreviouslyObservedAxes)
{
  std::unordered_map<std::string, double> values{
    {"axis_a/status_word", 0x0027}, {"axis_b/status_word", 0x0027}};
  auto message = makeSnapshot();
  message.joint_names.pop_back();
  message.interface_values.pop_back();

  ASSERT_TRUE(replaceDynamicStateSnapshot(message, values));

  EXPECT_EQ(values.count("axis_b/status_word"), 0U);
  EXPECT_EQ(values.at("axis_a/status_word"), 0x0027);
}

TEST(DynamicStateSnapshotTest, MalformedOrAmbiguousSnapshotsClearOldHealthData)
{
  auto malformed = makeSnapshot();
  malformed.interface_values[0].interface_names.push_back("status_word");
  malformed.interface_values[0].values.push_back(0x0027);
  std::unordered_map<std::string, double> values{{"axis_a/status_word", 0x0027}};

  EXPECT_FALSE(replaceDynamicStateSnapshot(malformed, values));
  EXPECT_TRUE(values.empty());

  auto mismatched = makeSnapshot();
  mismatched.interface_values.pop_back();
  values.emplace("axis_b/status_word", 0x0027);
  EXPECT_FALSE(replaceDynamicStateSnapshot(mismatched, values));
  EXPECT_TRUE(values.empty());
}

}  // namespace
}  // namespace rt_diagnostics
