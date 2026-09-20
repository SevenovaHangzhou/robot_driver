#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gravity_ff_controller/gravity_ff_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_support.hpp"
#include <algorithm>
#include <array>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using Controller = gravity_ff_controller::GravityFeedforwardController;
using CallbackReturn = controller_interface::CallbackReturn;

namespace
{
const std::string urdf =
  R"(<robot name="direct">
<link name="base"/><link name="link1"><inertial><origin xyz="0.5 0 0"/><mass value="2"/>
<inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial></link>
<joint name="joint1" type="revolute"><parent link="base"/><child link="link1"/><axis xyz="0 1 0"/>
<limit lower="-3" upper="3" effort="100" velocity="2"/></joint>
<link name="link2"><inertial><origin xyz="0.25 0 0"/><mass value="1"/>
<inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial></link>
<joint name="joint2" type="revolute"><parent link="link1"/><child link="link2"/><origin xyz="1 0 0"/>
<axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="100" velocity="2"/></joint>
<link name="tool"/><joint name="tool_mount" type="fixed"><parent link="link2"/><child link="tool"/></joint>
</robot>)";

class ControllerTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  Controller controller;
  std::array<double, 6> states{0.0, 64.0, 111.0, 0.0, 64.0, -222.0};
  std::array<double, 2> commands{0.0, 0.0};
  std::vector<hardware_interface::StateInterface> state_handles;
  std::vector<hardware_interface::CommandInterface> command_handles;

  void configure(bool active, bool verified)
  {
    ASSERT_EQ(
      controller.init("gravity_test"),
      controller_interface::return_type::OK);
    controller.get_node()->set_parameters(
      gravity_parameters(urdf, active, verified));
    ASSERT_EQ(
      controller.on_configure(rclcpp_lifecycle::State{}),
      CallbackReturn::SUCCESS);
  }
  void activate(bool active)
  {
    const auto state_names = controller.state_interface_configuration().names;
    ASSERT_TRUE(state_names.size() == 4U || state_names.size() == states.size());
    state_handles.clear();
    command_handles.clear();
    state_handles.reserve(state_names.size());
    command_handles.reserve(commands.size());
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    loaned_states.reserve(state_names.size());
    const std::array<std::string, 6> expected_names{
      "joint1/position", "joint1/status_word", "joint1/torque_actual_permille",
      "joint2/position", "joint2/status_word", "joint2/torque_actual_permille"};
    for (std::size_t i = 0U; i < state_names.size(); ++i) {
      const auto split = state_names[i].find_last_of('/');
      const auto value = std::find(
        expected_names.begin(), expected_names.end(), state_names[i]);
      ASSERT_NE(value, expected_names.end());
      state_handles.emplace_back(
        state_names[i].substr(0, split),
        state_names[i].substr(split + 1),
        &states[static_cast<std::size_t>(value - expected_names.begin())]);
      loaned_states.emplace_back(state_handles.back());
    }
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    loaned_commands.reserve(commands.size());
    if (active) {
      const auto command_names =
        controller.command_interface_configuration().names;
      ASSERT_EQ(command_names.size(), commands.size());
      for (std::size_t i = 0U; i < command_names.size(); ++i) {
        const auto split = command_names[i].find_last_of('/');
        command_handles.emplace_back(
          command_names[i].substr(0, split),
          command_names[i].substr(split + 1),
          &commands[i]);
        loaned_commands.emplace_back(command_handles.back());
      }
    }
    controller.assign_interfaces(
      std::move(loaned_commands),
      std::move(loaned_states));
    ASSERT_EQ(
      controller.on_activate(rclcpp_lifecycle::State{}),
      CallbackReturn::SUCCESS);
  }
};
} // namespace

TEST_F(
  ControllerTest,
  ShadowClaimsNoCommandsAndPublishesRawTorqueAndValidationFlags) {
  configure(false, false);
  EXPECT_EQ(
    controller.command_interface_configuration().type,
    controller_interface::interface_configuration_type::NONE);
  activate(false);
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr received;
  auto client = std::make_shared<rclcpp::Node>("gravity_diag_client");
  auto subscriber =
    client->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/gravity_test/state", 1,
    [&](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
      received = message;
    });
  (void)subscriber;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller.get_node()->get_node_base_interface());
  executor.add_node(client);
  controller.update(
    controller.get_node()->now(),
    rclcpp::Duration::from_seconds(0.1));
  for (int i = 0; i < 10 && !received; ++i) {
    executor.spin_some();
  }
  ASSERT_TRUE(received);
  ASSERT_EQ(received->status.size(), 1U);
  const auto & values = received->status[0].values;
  EXPECT_EQ(values[0].value, "false");
  EXPECT_EQ(values[1].value, "false");
  EXPECT_DOUBLE_EQ(std::stod(values[4].value), 111.0);
  EXPECT_DOUBLE_EQ(std::stod(values[7].value), -222.0);
}

TEST_F(ControllerTest, ShadowConfiguresWithoutEffortLimitsOrCalibration)
{
  ASSERT_EQ(controller.init("shadow_without_limits"), controller_interface::return_type::OK);
  auto parameters = gravity_parameters(urdf, false, false);
  parameters.erase(
    std::remove_if(
      parameters.begin(), parameters.end(), [](const auto & parameter) {
        return parameter.get_name() == "max_effort_nm" ||
        parameter.get_name() == "max_slew_nm_per_s";
      }), parameters.end());
  controller.get_node()->set_parameters(parameters);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_EQ(
    controller.command_interface_configuration().type,
    controller_interface::interface_configuration_type::NONE);
  activate(false);
  EXPECT_EQ(
    controller.update(
      controller.get_node()->now(),
      rclcpp::Duration::from_seconds(0.1)), controller_interface::return_type::OK);
}

TEST_F(ControllerTest, ActiveCanUsePositionAndStatusWithoutTorqueActual)
{
  ASSERT_EQ(
    controller.init("active_without_torque_actual"),
    controller_interface::return_type::OK);
  auto parameters = gravity_parameters(urdf, true, true);
  parameters.emplace_back("torque_actual_interface", "");
  controller.get_node()->set_parameters(parameters);
  ASSERT_EQ(
    controller.on_configure(rclcpp_lifecycle::State{}),
    CallbackReturn::SUCCESS);
  EXPECT_EQ(controller.state_interface_configuration().names.size(), 4U);
  activate(true);
  controller.update(
    controller.get_node()->now(),
    rclcpp::Duration::from_seconds(0.1));
  EXPECT_NE(commands[0], 0.0);
}

TEST_F(ControllerTest, RejectedParameterBatchDoesNotPartiallyUpdateScale)
{
  configure(true, true);
  activate(true);
  const auto rejected = controller.get_node()->set_parameters_atomically(
  {
    rclcpp::Parameter("scale", std::vector<double>{0.0, 0.0}),
    rclcpp::Parameter("mode", "shadow")});
  EXPECT_FALSE(rejected.successful);
  controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.1));
  EXPECT_NE(commands[0], 0.0);
  const auto wrong_type = controller.get_node()->set_parameters_atomically(
  {
    rclcpp::Parameter("scale", std::vector<std::string>{"bad", "bad"})});
  EXPECT_FALSE(wrong_type.successful);
}

TEST_F(ControllerTest, ActiveRejectsUnverifiedOrMismatchedModelAndCalibration) {
  Controller controller;
  ASSERT_EQ(
    controller.init("gravity_reject"),
    controller_interface::return_type::OK);
  controller.get_node()->set_parameters(gravity_parameters(urdf, true, false));
  EXPECT_EQ(
    controller.on_configure(rclcpp_lifecycle::State{}),
    CallbackReturn::ERROR);

  Controller mismatch;
  ASSERT_EQ(
    mismatch.init("gravity_hash_reject"),
    controller_interface::return_type::OK);
  auto parameters = gravity_parameters(urdf, true, true);
  for (auto & parameter : parameters) {
    if (parameter.get_name() == "model_validation.urdf_sha256") {
      parameter =
        rclcpp::Parameter(parameter.get_name(), std::string(64U, '0'));
    }
  }
  mismatch.get_node()->set_parameters(parameters);
  EXPECT_EQ(
    mismatch.on_configure(rclcpp_lifecycle::State{}),
    CallbackReturn::ERROR);

  Controller unsourced;
  ASSERT_EQ(
    unsourced.init("gravity_unsourced"),
    controller_interface::return_type::OK);
  auto unsourced_parameters = gravity_parameters(urdf, true, true);
  for (auto & parameter : unsourced_parameters) {
    if (parameter.get_name() == "effort_calibration.source") {
      parameter = rclcpp::Parameter(
        parameter.get_name(), std::vector<std::string>{"synthetic", "TBD"});
    }
  }
  unsourced.get_node()->set_parameters(unsourced_parameters);
  EXPECT_EQ(
    unsourced.on_configure(rclcpp_lifecycle::State{}),
    CallbackReturn::ERROR);
}

TEST_F(
  ControllerTest,
  ActiveWritesEffortSupportsScaleUpdateAndZerosOnDeactivate) {
  configure(true, true);
  activate(true);
  controller.update(
    controller.get_node()->now(),
    rclcpp::Duration::from_seconds(0.1));
  EXPECT_NE(commands[0], 0.0);
  EXPECT_NE(commands[1], 0.0);
  const auto result = controller.get_node()->set_parameter(
    rclcpp::Parameter("scale", std::vector<double>{0.0, 0.0}));
  ASSERT_TRUE(result.successful);
  for (int i = 0; i < 10; ++i) {
    controller.update(
      controller.get_node()->now(),
      rclcpp::Duration::from_seconds(0.1));
  }
  EXPECT_DOUBLE_EQ(commands[0], 0.0);
  EXPECT_DOUBLE_EQ(commands[1], 0.0);
  controller.get_node()->set_parameter(
    rclcpp::Parameter("scale", std::vector<double>{1.0, 1.0}));
  controller.update(
    controller.get_node()->now(),
    rclcpp::Duration::from_seconds(0.1));
  ASSERT_NE(commands[0], 0.0);
  EXPECT_EQ(
    controller.on_deactivate(rclcpp_lifecycle::State{}),
    CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(commands[0], 0.0);
  EXPECT_DOUBLE_EQ(commands[1], 0.0);
}

TEST_F(
  ControllerTest,
  InvalidEnabledFeedbackSlewsToZeroAndLatchClearsOnlyOnReactivate) {
  configure(true, true);
  activate(true);
  states[1] = states[4] = 39.0;
  controller.update(
    controller.get_node()->now(),
    rclcpp::Duration::from_seconds(0.1));
  const double before = commands[0];
  ASSERT_NE(before, 0.0);
  states[0] = std::numeric_limits<double>::quiet_NaN();
  controller.update(
    controller.get_node()->now(),
    rclcpp::Duration::from_seconds(0.001));
  EXPECT_LT(std::abs(commands[0]), std::abs(before));
  EXPECT_NE(commands[0], 0.0);
  states[0] = 0.0;
  controller.update(
    controller.get_node()->now(),
    rclcpp::Duration::from_seconds(1.0));
  EXPECT_DOUBLE_EQ(commands[0], 0.0);
  controller.update(
    controller.get_node()->now(),
    rclcpp::Duration::from_seconds(1.0));
  EXPECT_DOUBLE_EQ(commands[0], 0.0);
  controller.on_deactivate(rclcpp_lifecycle::State{});
  controller.release_interfaces();
  activate(true);
  controller.update(
    controller.get_node()->now(),
    rclcpp::Duration::from_seconds(0.1));
  EXPECT_NE(commands[0], 0.0);
}
