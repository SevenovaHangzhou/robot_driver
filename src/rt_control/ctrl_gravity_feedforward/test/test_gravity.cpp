#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ctrl_gravity_feedforward/gravity_controller.hpp"
#include "ctrl_gravity_feedforward/gravity_model.hpp"
#include "hardware_interface/handle.hpp"
#include "pluginlib/class_loader.hpp"

namespace
{
using Model = ctrl_gravity_feedforward::GravityModel;
using Controller = ctrl_gravity_feedforward::GravityController;
using Callback = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

// Both tips are massless. The massive tool is a sibling of tip, not on its chain.
const std::string robot = R"(
<robot name="pendulum">
  <link name="base"/>
  <link name="rod"><inertial><origin xyz="0.5 0 0"/><mass value="2"/>
    <inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/>
  </inertial></link>
  <joint name="hinge" type="continuous"><parent link="base"/><child link="rod"/>
    <axis xyz="0 1 0"/><limit effort="100" velocity="1"/></joint>
  <link name="tool"><inertial><mass value="3"/>
    <inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/>
  </inertial></link>
  <joint name="tool_mount" type="fixed"><parent link="rod"/><child link="tool"/>
    <origin xyz="1 0 0"/></joint>
  <link name="tip"/>
  <joint name="tip_mount" type="fixed"><parent link="rod"/><child link="tip"/>
    <origin xyz="2 0 0"/></joint>
</robot>)";

TEST(GravityModel, IncludesSiblingToolAndUsesUrdfTorqueSign)
{
  Model model(robot, "base", "tip", {0.0, 0.0, -9.81});
  EXPECT_EQ(model.joints(), std::vector<std::string>{"hinge"});
  ASSERT_TRUE(model.compute({0.0}));
  EXPECT_NEAR(model.torques()[0], -39.24, 1e-12);
  ASSERT_TRUE(model.compute({std::acos(-1.0) / 2.0}));
  EXPECT_NEAR(model.torques()[0], 0.0, 1e-12);
  ASSERT_TRUE(model.compute({std::acos(-1.0)}));
  EXPECT_NEAR(model.torques()[0], 39.24, 1e-12);
  // Ending at the last moving link must retain its fixed descendants as well.
  Model without_tcp(robot, "base", "rod", {0.0, 0.0, -9.81});
  ASSERT_TRUE(without_tcp.compute({0.0}));
  EXPECT_NEAR(without_tcp.torques()[0], -39.24, 1e-12);
}

TEST(GravityModel, ArbitraryGravityAndInvalidInputs)
{
  Model model(robot, "base", "tip", {9.81, 0.0, 0.0});
  ASSERT_TRUE(model.compute({std::acos(-1.0) / 2.0}));
  EXPECT_NEAR(model.torques()[0], 39.24, 1e-12);
  EXPECT_FALSE(model.compute({std::numeric_limits<double>::quiet_NaN()}));
  EXPECT_EQ(model.torques()[0], 0.0);
  EXPECT_FALSE(model.compute({}));
  EXPECT_FALSE(model.compute({0.0, 0.0}));
  Model zero(robot, "base", "tip", {0.0, 0.0, 0.0});
  ASSERT_TRUE(zero.compute({1.0}));
  EXPECT_EQ(zero.torques()[0], 0.0);
  EXPECT_THROW(Model(robot, "tip", "base", {0.0, 0.0, -9.81}), std::invalid_argument);
}

TEST(GravityModel, RejectsUnmodelledMovingBranch)
{
  auto branched = robot;
  const std::string from = "name=\"tool_mount\" type=\"fixed\"";
  branched.replace(branched.find(from), from.size(), "name=\"tool_mount\" type=\"continuous\"");
  EXPECT_THROW(Model(branched, "base", "tip", {0.0, 0.0, -9.81}), std::invalid_argument);
}

class GravityControllerTest : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  Controller controller;
  double command{123.0};
  // position, CiA402 status, feedback age, link state, slave AL state.
  std::array<double, 5> values{0.0, 39.0, 0.0, 1.0, 8.0};
  std::vector<hardware_interface::StateInterface> states;
  hardware_interface::CommandInterface effort{"hinge", "effort", &command};

  void configure()
  {
    ASSERT_EQ(controller.init("gravity_test"), Return::OK);
    controller.get_node()->set_parameters(
      {{"robot_description", robot},
       {"base_link", "base"},
       {"tip_link", "tip"},
       {"joints", std::vector<std::string>{"hinge"}},
       {"gravity_in_base", std::vector<double>{0.0, 0.0, -9.81}},
       {"max_effort_nm", std::vector<double>{10.0}},
       {"max_effort_rate_nm_s", std::vector<double>{100.0}},
       {"max_feedback_age_ms", 5.0},
       {"feedback_age_interface", "bus/age_ms"},
       {"health_interfaces", std::vector<std::string>{"bus/link", "drive/al_state"}},
       {"health_values", std::vector<double>{1.0, 8.0}}});
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  }

  void bind(bool complete = true)
  {
    const auto names = controller.state_interface_configuration().names;
    ASSERT_EQ(names.size(), values.size());
    states.reserve(names.size());
    for (std::size_t index = 0; index < names.size(); ++index) {
      const auto split = names[index].find_last_of('/');
      states.emplace_back(
        names[index].substr(0, split), names[index].substr(split + 1), &values[index]);
    }
    std::vector<hardware_interface::LoanedStateInterface> loaned;
    for (auto it = states.rbegin(); it != states.rend(); ++it) {
      loaned.emplace_back(*it);
    }
    if (!complete) {
      loaned.pop_back();
    }
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(effort);
    controller.assign_interfaces(std::move(commands), std::move(loaned));
  }

  Return cycle(double seconds = 0.001)
  {
    return controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(seconds));
  }
};

TEST_F(GravityControllerTest, ClaimsOnlyEffortRampsAndClamps)
{
  configure();
  EXPECT_EQ(
    controller.command_interface_configuration().names, std::vector<std::string>{"hinge/effort"});
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  EXPECT_EQ(command, 0.0);
  ASSERT_EQ(cycle(), Return::OK);
  EXPECT_NEAR(command, -0.1, 1e-12);
  for (int index = 0; index < 150; ++index) {
    ASSERT_EQ(cycle(), Return::OK);
  }
  EXPECT_EQ(command, -10.0);
  EXPECT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  EXPECT_EQ(command, 0.0);
  EXPECT_EQ(cycle(), Return::OK);
}

TEST_F(GravityControllerTest, RejectsPartialAndNotReadyActivation)
{
  configure();
  bind(false);
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::ERROR);
  EXPECT_EQ(command, 0.0);
}

TEST_F(GravityControllerTest, DisabledDriveCannotActivate)
{
  configure();
  bind();
  values[1] = 64.0;
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::ERROR);
  EXPECT_EQ(command, 0.0);
}

TEST_F(GravityControllerTest, StaleFeedbackFaultLatchesUntilLifecycleReset)
{
  configure();
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  ASSERT_EQ(cycle(), Return::OK);
  values[2] = 5.1;
  EXPECT_EQ(cycle(), Return::ERROR);
  EXPECT_EQ(command, 0.0);
  values[2] = 0.0;
  EXPECT_EQ(cycle(), Return::ERROR);
  EXPECT_EQ(command, 0.0);
  EXPECT_EQ(controller.on_error(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  EXPECT_EQ(command, 0.0);
  EXPECT_TRUE(controller.command_interface_configuration().names.empty());
}

TEST_F(GravityControllerTest, LinkLossDisableAndNonfiniteFeedbackZeroCommands)
{
  configure();
  bind();
  for (const auto & change : std::vector<std::pair<std::size_t, double>>{
         {0, std::numeric_limits<double>::infinity()},
         {1, 64.0},
         {1, 8.0},
         {2, -1.0},
         {3, 0.0},
         {4, 4.0}}) {
    values = {0.0, 39.0, 0.0, 1.0, 8.0};
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
    ASSERT_EQ(cycle(), Return::OK);
    values[change.first] = change.second;
    EXPECT_EQ(cycle(), Return::ERROR);
    EXPECT_EQ(command, 0.0);
  }
}

TEST_F(GravityControllerTest, ShutdownAndCleanupZeroCommands)
{
  configure();
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  ASSERT_EQ(cycle(), Return::OK);
  EXPECT_EQ(controller.on_shutdown(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  EXPECT_EQ(command, 0.0);
  EXPECT_EQ(controller.on_cleanup(rclcpp_lifecycle::State{}), Callback::SUCCESS);
}

TEST_F(GravityControllerTest, MissingLimitsAndWrongJointOrderRejectConfigure)
{
  configure();
  controller.get_node()->set_parameter(rclcpp::Parameter("max_effort_nm", std::vector<double>{}));
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), Callback::ERROR);
  EXPECT_TRUE(controller.command_interface_configuration().names.empty());
  controller.get_node()->set_parameter(
    rclcpp::Parameter("max_effort_nm", std::vector<double>{10.0}));
  controller.get_node()->set_parameter(
    rclcpp::Parameter("joints", std::vector<std::string>{"other"}));
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), Callback::ERROR);
}

TEST_F(GravityControllerTest, InvalidPeriodZerosAndLatches)
{
  configure();
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  ASSERT_EQ(cycle(), Return::OK);
  EXPECT_EQ(cycle(0.0), Return::ERROR);
  EXPECT_EQ(command, 0.0);
}

TEST_F(GravityControllerTest, PluginLoads)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_NE(loader.createSharedInstance("ctrl_gravity_feedforward/GravityController"), nullptr);
}
}  // namespace
