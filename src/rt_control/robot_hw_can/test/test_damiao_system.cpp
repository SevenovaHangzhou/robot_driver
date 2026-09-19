#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "robot_hw_can/damiao_system.hpp"

namespace robot_hw_can
{
namespace
{

[[nodiscard]] hardware_interface::HardwareInfo valid_info()
{
  hardware_interface::HardwareInfo info;
  info.name = "damiao_head";
  info.type = "system";
  info.hardware_class_type = "robot_hw_can/DamiaoSystem";
  info.hardware_parameters = {
    {"can_interface", "pciecan2"},
    {"configure_timeout_ms", "100"},
    {"feedback_timeout_ms", "100"},
    {"disabled_poll_interval_ms", "50"},
    {"max_rx_frames_per_cycle", "32"},
  };
  for (int index = 0; index < 2; ++index) {
    hardware_interface::ComponentInfo joint;
    joint.name = "head_motor_" + std::to_string(index + 1) + "_joint";
    joint.type = "joint";
    hardware_interface::InterfaceInfo command{"position"};
    command.min = "-0.1";
    command.max = "0.1";
    joint.command_interfaces.push_back(command);
    for (const char * state : {
      "position", "velocity", "effort", "fault_code", "mos_temperature",
      "motor_temperature", "feedback_age_ms", "enabled"})
    {
      joint.state_interfaces.push_back(hardware_interface::InterfaceInfo{state});
    }
    joint.parameters = {
      {"can_id", std::to_string(index + 1)},
      {"master_id", std::to_string(index + 17)},
      {"velocity_limit", "0.1"}};
    info.joints.push_back(std::move(joint));
  }
  return info;
}

TEST(DamiaoSystemTest, ExportsStandardControllerInterfacesWithoutOpeningCan)
{
  DamiaoSystem system;
  ASSERT_EQ(system.on_init(valid_info()), hardware_interface::CallbackReturn::SUCCESS);
  const auto states = system.export_state_interfaces();
  const auto commands = system.export_command_interfaces();
  EXPECT_EQ(states.size(), 16U);
  ASSERT_EQ(commands.size(), 2U);
  EXPECT_EQ(commands[0].get_name(), "head_motor_1_joint/position");
  EXPECT_EQ(commands[1].get_name(), "head_motor_2_joint/position");
  EXPECT_TRUE(std::isnan(commands[0].get_value()));
}

TEST(DamiaoSystemTest, RejectsPartialControllerClaims)
{
  DamiaoSystem system;
  ASSERT_EQ(system.on_init(valid_info()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.prepare_command_mode_switch({"head_motor_1_joint/position"}, {}),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.prepare_command_mode_switch({"head_motor_1_joint/position",
    "head_motor_2_joint/position"}, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(system.prepare_command_mode_switch({"other_joint/position"}, {}),
    hardware_interface::return_type::OK);
  EXPECT_EQ(system.perform_command_mode_switch({"head_motor_1_joint/position",
    "head_motor_2_joint/position"}, {}), hardware_interface::return_type::ERROR);
}

TEST(DamiaoSystemTest, RejectsInvalidIdsLimitsAndInterfaces)
{
  auto info = valid_info();
  DamiaoSystem system;
  info.joints[1].parameters["master_id"] = "17";
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
  info = valid_info();
  info.joints[1].parameters["can_id"] = "16";
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
  info = valid_info();
  info.joints[0].command_interfaces[0].min.clear();
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
  info = valid_info();
  info.joints[0].state_interfaces.pop_back();
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
  info = valid_info();
  info.hardware_parameters.erase("feedback_timeout_ms");
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
}

TEST(DamiaoSystemTest, MissingCanInterfaceFailsConfigureWithoutEnabling)
{
  auto info = valid_info();
  info.hardware_parameters["can_interface"] = "no_such_can_if";
  DamiaoSystem system;
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(system.on_cleanup(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
}

}  // namespace
}  // namespace robot_hw_can
