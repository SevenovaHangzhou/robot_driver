#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "hardware_interface/resource_manager.hpp"

TEST(KincoCyclicModeGenericSystem, ExposesTheAdapterControllerContractWithoutControlwordAliasing)
{
  const std::string urdf =
    R"(
<robot name="kinco_mode_contract">
  <ros2_control name="kinco_mode_mock" type="system">
    <hardware><plugin>mock_components/GenericSystem</plugin></hardware>
    <joint name="drive_axis">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="mode_of_operation"/>
      <command_interface name="control_word"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="mode_of_operation_display"/>
      <state_interface name="status_word"/>
      <state_interface name="feedback_age_ms"/>
      <state_interface name="mode_switch_ready"/>
      <state_interface name="mode_request_ack"/>
      <state_interface name="command_fresh"/>
      <state_interface name="mode_request_error"/>
    </joint>
  </ros2_control>
</robot>)";
  hardware_interface::ResourceManager resources(urdf, true, true);
  auto commands = resources.command_interface_keys();
  auto states = resources.state_interface_keys();
  std::sort(commands.begin(), commands.end());
  std::sort(states.begin(), states.end());

  EXPECT_EQ(
    commands, (std::vector<std::string>{
    "drive_axis/control_word", "drive_axis/mode_of_operation",
    "drive_axis/position", "drive_axis/velocity"}));
  EXPECT_EQ(
    states, (std::vector<std::string>{
    "drive_axis/command_fresh", "drive_axis/feedback_age_ms",
    "drive_axis/mode_of_operation_display", "drive_axis/mode_request_ack",
    "drive_axis/mode_request_error", "drive_axis/mode_switch_ready",
    "drive_axis/position", "drive_axis/status_word", "drive_axis/velocity"}));

  auto mode = resources.claim_command_interface("drive_axis/mode_of_operation");
  auto velocity = resources.claim_command_interface("drive_axis/velocity");
  auto control_word = resources.claim_command_interface("drive_axis/control_word");
  mode.set_value(8.0);
  velocity.set_value(0.0);
  control_word.set_value(0x000f);
  EXPECT_DOUBLE_EQ(mode.get_value(), 8.0);
  EXPECT_DOUBLE_EQ(velocity.get_value(), 0.0);
  EXPECT_DOUBLE_EQ(control_word.get_value(), 0x000f);
}
