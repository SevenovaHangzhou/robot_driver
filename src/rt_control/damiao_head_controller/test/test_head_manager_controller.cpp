#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include "damiao_head_controller/head_manager_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace damiao_head_controller
{
namespace
{

using CallbackReturn = controller_interface::CallbackReturn;

class HeadManagerControllerTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  HeadManagerController controller;
  std::array<double, 2U> commands{};
  std::array<double, 20U> states{};
  std::vector<hardware_interface::CommandInterface> command_handles;
  std::vector<hardware_interface::StateInterface> state_handles;

  void SetUp() override
  {
    ASSERT_EQ(controller.init("damiao_head_manager_test"), controller_interface::return_type::OK);
    controller.get_node()->set_parameters({
      rclcpp::Parameter("joints", std::vector<std::string>{"head_joint", "head_pitch_joint"}),
      rclcpp::Parameter("control_name", "damiao_head_control"),
      rclcpp::Parameter("position_controller_name", "head_position_controller"),
      rclcpp::Parameter("service_prefix", "/rt/head"),
      rclcpp::Parameter("service_timeout_ms", 500),
      rclcpp::Parameter("controller_switch_timeout_ms", 100),
    });
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);

    const auto command_names = controller.command_interface_configuration().names;
    const auto state_names = controller.state_interface_configuration().names;
    ASSERT_EQ(command_names.size(), commands.size());
    ASSERT_EQ(state_names.size(), states.size());
    for (std::size_t index{0U}; index < command_names.size(); ++index) {
      const auto slash = command_names[index].find_last_of('/');
      command_handles.emplace_back(
        command_names[index].substr(0U, slash), command_names[index].substr(slash + 1U),
        &commands[index]);
    }
    for (std::size_t index{0U}; index < state_names.size(); ++index) {
      const auto slash = state_names[index].find_last_of('/');
      state_handles.emplace_back(
        state_names[index].substr(0U, slash), state_names[index].substr(slash + 1U),
        &states[index]);
    }
    states[0] = static_cast<double>(static_cast<std::uint8_t>(HeadPhase::disabled));
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    for (auto & handle : command_handles) {
      loaned_commands.emplace_back(handle);
    }
    for (auto & handle : state_handles) {
      loaned_states.emplace_back(handle);
    }
    controller.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    static_cast<void>(controller.on_deactivate(rclcpp_lifecycle::State{}));
  }
};

TEST_F(HeadManagerControllerTest, ClaimsOnlyLifecycleCommandsAndAllDiagnosticStates)
{
  const auto commands_config = controller.command_interface_configuration();
  EXPECT_EQ(commands_config.names, (std::vector<std::string>{
    "damiao_head_control/enable_request", "damiao_head_control/reset_generation"}));
  const auto states_config = controller.state_interface_configuration();
  EXPECT_EQ(states_config.names[0], "damiao_head_control/phase");
  EXPECT_EQ(states_config.names[4], "head_joint/position");
  EXPECT_EQ(states_config.names[12], "head_pitch_joint/position");
}

TEST_F(HeadManagerControllerTest, ActivationStartsDisabledWithoutIssuingAReset)
{
  EXPECT_EQ(
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.001)),
    controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(commands[0], 0.0);
  EXPECT_DOUBLE_EQ(commands[1], 0.0);
}

TEST_F(HeadManagerControllerTest, InvalidTopologyIsRejected)
{
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  controller.get_node()->set_parameter(
    rclcpp::Parameter("joints", std::vector<std::string>{"head_joint"}));
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}

}  // namespace
}  // namespace damiao_head_controller
