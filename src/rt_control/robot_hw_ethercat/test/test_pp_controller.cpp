#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "gripper_controllers/gripper_action_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_hw_ethercat/zeroerr_pp_slave.hpp"

using Controller = gripper_action_controller::GripperActionController<
  hardware_interface::HW_IF_POSITION>;
using Action = control_msgs::action::GripperCommand;
using CallbackReturn = controller_interface::CallbackReturn;

class PpControllerTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  Controller controller;
  std::vector<double> command{0.0, 0.0, 0.0, 1.0, 0.0};
  std::vector<double> state{0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  std::function<void()> hardware_cycle;
  std::vector<hardware_interface::CommandInterface> command_handles;
  std::vector<hardware_interface::StateInterface> state_handles;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr client_node;
  rclcpp_action::Client<Action>::SharedPtr client;

  void SetUp() override
  {
    ASSERT_EQ(controller.init("test_pp_gripper"), controller_interface::return_type::OK);
    controller.get_node()->set_parameters({
      {"joint", "gripper"}, {"pp.enabled", true}, {"pp.min_position", 0.0},
      {"pp.max_position", 0.08}, {"max_effort", 10.0}, {"pp.command_timeout", 0.5},
      {"goal_tolerance", 0.0001}, {"stall_velocity_threshold", 0.0001},
      {"stall_timeout", 0.2}, {"action_monitor_rate", 100.0}});
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    const std::array<std::string, 4> commands{
      "position", "max_effort", "pp_sequence", "pp_halt"};
    const std::array<std::string, 5> states{
      "position", "velocity", "pp_sequence", "pp_state", "effort"};
    for (size_t i = 0; i < commands.size(); ++i) {
      command_handles.emplace_back("gripper", commands[i], &command[i]);
    }
    for (size_t i = 0; i < states.size(); ++i) {
      state_handles.emplace_back("gripper", states[i], &state[i]);
    }
    std::vector<hardware_interface::LoanedCommandInterface> commands_loaned;
    std::vector<hardware_interface::LoanedStateInterface> states_loaned;
    for (auto & handle : command_handles) {commands_loaned.emplace_back(handle);}
    for (auto & handle : state_handles) {states_loaned.emplace_back(handle);}
    controller.assign_interfaces(std::move(commands_loaned), std::move(states_loaned));
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    executor.add_node(controller.get_node()->get_node_base_interface());
    client_node = std::make_shared<rclcpp::Node>("pp_gripper_test_client");
    executor.add_node(client_node);
    client = rclcpp_action::create_client<Action>(client_node, "/test_pp_gripper/gripper_cmd");
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
    cycle();
  }

  void TearDown() override {controller.on_deactivate(rclcpp_lifecycle::State{});}

  void cycle()
  {
    executor.spin_some();
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    if (hardware_cycle) {hardware_cycle();}
  }

  template<typename Future>
  bool wait(Future & future)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < end) {
      cycle();
      if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {return true;}
    }
    return false;
  }

  auto goal(double position = 0.04, double force = 5.0)
  {
    Action::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = force;
    auto future = client->async_send_goal(goal);
    if (!wait(future)) {return rclcpp_action::ClientGoalHandle<Action>::SharedPtr{};}
    return future.get();
  }
};

TEST_F(PpControllerTest, ClaimsLimitAndHandshakeButNeverControlWord)
{
  EXPECT_EQ(controller.command_interface_configuration().names,
    (std::vector<std::string>{"gripper/position", "gripper/max_effort",
      "gripper/pp_sequence", "gripper/pp_halt"}));
  EXPECT_EQ(command[2], 0.0);
  EXPECT_EQ(command[3], 1.0);
}

TEST_F(PpControllerTest, ValidatesLimitsAndRejectsBusyGoals)
{
  EXPECT_FALSE(goal(0.09));
  EXPECT_FALSE(goal(0.04, 11.0));
  EXPECT_FALSE(goal(0.04, 0.0));
  ASSERT_TRUE(goal());
  cycle();
  EXPECT_EQ(command[0], 0.04);
  EXPECT_EQ(command[1], 5.0);
  EXPECT_GT(command[2], 0.0);
  EXPECT_EQ(command[3], 0.0);
  EXPECT_FALSE(goal());
}

TEST_F(PpControllerTest, RequiresHardwareCompletionForTheSameSequence)
{
  const auto handle = goal();
  ASSERT_TRUE(handle);
  auto result = client->async_get_result(handle);
  state[0] = 0.04;
  state[3] = 6.0;
  cycle();
  EXPECT_NE(result.wait_for(std::chrono::seconds(0)), std::future_status::ready);
  state[2] = command[2];
  state[4] = 2.0;
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.get().result->effort, 2.0);
  EXPECT_TRUE(result.get().result->reached_goal);
}

TEST_F(PpControllerTest, CancelWaitsForHardwareHalt)
{
  const auto handle = goal();
  ASSERT_TRUE(handle);
  auto result = client->async_get_result(handle);
  auto canceled = client->async_cancel_goal(handle);
  ASSERT_TRUE(wait(canceled));
  cycle();
  EXPECT_EQ(command[3], 1.0);
  EXPECT_EQ(command[1], 5.0);
  EXPECT_NE(result.wait_for(std::chrono::seconds(0)), std::future_status::ready);
  state[2] = command[2];
  state[3] = 8.0;
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(PpControllerTest, FaultAbortsAndInactiveControllerRejectsGoals)
{
  const auto handle = goal();
  ASSERT_TRUE(handle);
  auto result = client->async_get_result(handle);
  state[3] = -1.0;
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(command[3], 1.0);
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_FALSE(goal());
}

TEST_F(PpControllerTest, FaultDuringCancelAbortsInsteadOfHanging)
{
  const auto handle = goal();
  ASSERT_TRUE(handle);
  auto result = client->async_get_result(handle);
  auto cancel = client->async_cancel_goal(handle);
  ASSERT_TRUE(wait(cancel));
  state[3] = -1.0;
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(PpControllerTest, BusLossAndCommandTimeoutAbort)
{
  const auto handle = goal();
  ASSERT_TRUE(handle);
  auto result = client->async_get_result(handle);
  controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(1.0));
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  state[3] = 0.0;
  cycle();
  EXPECT_FALSE(goal());
}

TEST_F(PpControllerTest, ReactivationContinuesTheHardwareSequence)
{
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  std::vector<hardware_interface::LoanedCommandInterface> cmds;
  std::vector<hardware_interface::LoanedStateInterface> sts;
  for (auto & handle : command_handles) {cmds.emplace_back(handle);}
  for (auto & handle : state_handles) {sts.emplace_back(handle);}
  controller.assign_interfaces(std::move(cmds), std::move(sts));
  state[2] = 42.0;
  state[3] = 8.0;
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  cycle();
  ASSERT_TRUE(goal());
  cycle();
  EXPECT_GT(command[2], 42.0);
}

TEST_F(PpControllerTest, ActionToRawPdoAndBackCompletesOnlyAfterHandshake)
{
  robot_hw_ethercat::ZeroErrPpSlave slave;
  std::unordered_map<std::string, std::string> parameters{
    {"slave_config", PP_FIXTURE},
    {"command_interface/position", "0"}, {"command_interface/max_effort", "1"},
    {"command_interface/pp_sequence", "2"}, {"command_interface/pp_halt", "3"},
    {"command_interface/control_word", "4"},
    {"state_interface/position", "0"}, {"state_interface/velocity", "1"},
    {"state_interface/effort", "4"}, {"state_interface/status_word", "5"},
    {"state_interface/pp_sequence", "2"}, {"state_interface/pp_state", "3"}};
  ASSERT_TRUE(slave.setupSlave(parameters, &state, &command));
  slave.set_state_is_operational(true);
  std::array<std::array<uint8_t, 4>, 8> pdo{};
  EC_WRITE_U16(pdo[3].data(), 0x0027);
  EC_WRITE_S8(pdo[4].data(), 1);
  EC_WRITE_S32(pdo[5].data(), 100);
  hardware_cycle = [&]()
    {
      slave.onPdoCycleRead(true);
      for (size_t i = 0; i < pdo.size(); ++i) {slave.processData(i, pdo[i].data());}
      slave.onPdoCycleStart(true);
      for (size_t i = 0; i < pdo.size(); ++i) {slave.processData(i, pdo[i].data());}
      slave.onPdoCycleSent();
      const bool trigger = (EC_READ_U16(pdo[0].data()) & 0x0010) != 0;
      EC_WRITE_U16(pdo[3].data(), trigger ? 0x1427 : 0x0427);
      if (trigger) {EC_WRITE_S32(pdo[5].data(), EC_READ_S32(pdo[2].data()));}
    };
  cycle();
  command[4] = 0x000f;
  cycle();
  cycle();
  const auto handle = goal();
  ASSERT_TRUE(handle);
  auto result = client->async_get_result(handle);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_DOUBLE_EQ(result.get().result->position, 0.04);
  EXPECT_EQ(EC_READ_U16(pdo[1].data()), 100);
  EXPECT_EQ(command[4], 0x000f);
  hardware_cycle = {};
}

TEST_F(PpControllerTest, StallStopsBeforeReportingAborted)
{
  const auto handle = goal();
  ASSERT_TRUE(handle);
  auto result = client->async_get_result(handle);
  state[2] = command[2];
  state[3] = 5.0;
  controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.3));
  EXPECT_EQ(command[3], 1.0);
  EXPECT_NE(result.wait_for(std::chrono::seconds(0)), std::future_status::ready);
  state[3] = 8.0;
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_TRUE(result.get().result->stalled);
}

TEST_F(PpControllerTest, MissingLimitsAndHandshakeInterfacesPreventActivation)
{
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  controller.get_node()->set_parameter({"pp.max_position", 0.0});
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
  controller.get_node()->set_parameter({"pp.max_position", 0.08});
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  std::vector<hardware_interface::LoanedCommandInterface> cmds;
  std::vector<hardware_interface::LoanedStateInterface> sts;
  cmds.emplace_back(command_handles[0]);
  for (auto & handle : state_handles) {sts.emplace_back(handle);}
  controller.assign_interfaces(std::move(cmds), std::move(sts));
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}
