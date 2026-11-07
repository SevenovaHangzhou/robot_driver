// Copyright 2026 kkozia
// Licensed under the Apache License, Version 2.0
//
// Velocity-mode group (the swerve chassis): enable_manager switches the drive wheels between their
// CSV startup mode under the swerve controller and CSP under the wheel trajectory controller.

#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ctrl_enable_manager/enable_manager_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_rt_control_interfaces/msg/joint_control_mode.hpp"
#include "robot_system_interfaces/msg/error_code.hpp"

namespace ctrl_enable_manager
{

class EnableManagerTestAccess
{
public:
  using Controller = EnableManagerController;
  using Phase = Controller::Phase;
  using Owner = Controller::Owner;
  using ControlMode = Controller::ControlMode;
  using ModeService = Controller::ModeService;

  static Phase phase(const Controller & c) { return c.phase_.load(); }
  static void setPhase(Controller & c, Phase p) { c.phase_.store(p); }
  static Owner owner(const Controller & c) { return c.owner_.load(); }
  static void setOwner(Controller & c, Owner o) { c.owner_.store(o); }
  static ControlMode controlMode(const Controller & c) { return c.current_control_mode_.load(); }
  static void setControlMode(Controller & c, ControlMode m) { c.current_control_mode_.store(m); }
  static void requestCsp(Controller & c, bool csp) { c.drive_csp_requested_.store(csp); }
  static bool cspRequested(const Controller & c) { return c.drive_csp_requested_.load(); }
  static bool disableRequested(const Controller & c) { return c.disable_request_.load(); }
  static ControlMode detectControlMode(
    const Controller & c, const controller_manager_msgs::srv::ListControllers::Response & states)
  {
    return c.detectControlMode(states);
  }
  static void handleSetMode(
    Controller & c, std::shared_ptr<ModeService::Request> request,
    std::shared_ptr<ModeService::Response> response)
  {
    c.handleSetMode(request, response);
  }
  static bool activateMotionControllers(Controller & c)
  {
    return c.switchJtc(true) == Controller::SwitchResult::kSuccess;
  }
  static void handleJtcState(
    Controller & c, const control_msgs::msg::JointTrajectoryControllerState::SharedPtr & message)
  {
    c.handleJtcState(message);
  }
};

namespace
{

using Access = EnableManagerTestAccess;
using Phase = Access::Phase;
using Owner = Access::Owner;
using ControlMode = Access::ControlMode;
using ModeMessage = robot_rt_control_interfaces::msg::JointControlMode;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using SwitchController = controller_manager_msgs::srv::SwitchController;

constexpr double kOperationEnabled = 0x0027;
constexpr double kSwitchOnDisabled = 0x0040;
const std::vector<std::string> kJoints{"steer_joint", "wheel_a_joint", "wheel_b_joint"};
const std::vector<std::string> kWheels{"wheel_a_joint", "wheel_b_joint"};

std::vector<rclcpp::Parameter> baseParameters()
{
  return {
    rclcpp::Parameter("managed_joints", kJoints),
    rclcpp::Parameter("enable_batch_joint_names", kJoints),
    rclcpp::Parameter("enable_batch_sizes", std::vector<std::int64_t>{3}),
    rclcpp::Parameter("disable_terminal_policy", std::string("switch_on_disabled")),
    rclcpp::Parameter("expected_modes", std::vector<std::int64_t>{8, 9, 9}),
    rclcpp::Parameter("motion_controller", std::string("swerve_controller")),
    rclcpp::Parameter("trajectory_controller", std::string("wheel_jtc")),
    rclcpp::Parameter("mode_service", std::string("/rt/base/set_control_mode")),
    rclcpp::Parameter("motion_joints", kJoints),
    rclcpp::Parameter("mode_switch_joints", kWheels),
    rclcpp::Parameter(
      "mode_switch_stable_velocity_thresholds", std::vector<double>{0.01, 0.1, 0.1}),
    rclcpp::Parameter("mode_switch_takeover_tolerances", std::vector<double>(3U, 0.0087)),
    rclcpp::Parameter("controller_switch_timeout", 0.05),
    rclcpp::Parameter("update_rate", 1000),
  };
}

void replace(std::vector<rclcpp::Parameter> & parameters, const rclcpp::Parameter & candidate)
{
  for (auto & parameter : parameters) {
    if (parameter.get_name() == candidate.get_name()) {
      parameter = candidate;
      return;
    }
  }
  parameters.push_back(candidate);
}

class DriveModeFixture : public ::testing::Test
{
public:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void TearDown() override
  {
    controller_->release_interfaces();
    controller_.reset();
  }

  controller_interface::CallbackReturn configure(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    auto parameters = baseParameters();
    for (const auto & parameter : overrides) {
      replace(parameters, parameter);
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    controller_ = std::make_unique<EnableManagerController>();
    EXPECT_EQ(
      controller_->init("drive_mode_test", "", options), controller_interface::return_type::OK);
    return controller_->on_configure(rclcpp_lifecycle::State());
  }

  // Loans one buffer per configured interface, then walks the startup sanitize phase to IDLE.
  void bringUpToIdle()
  {
    ASSERT_EQ(configure(), controller_interface::CallbackReturn::SUCCESS);
    const auto commands = controller_->command_interface_configuration().names;
    const auto states = controller_->state_interface_configuration().names;
    command_handles_.reserve(commands.size());
    state_handles_.reserve(states.size());
    for (const auto & name : commands) {
      values_[name] = std::nan("");
      const auto slash = name.find('/');
      command_handles_.emplace_back(name.substr(0, slash), name.substr(slash + 1U), &values_[name]);
    }
    for (const auto & name : states) {
      values_[name] = 0.0;
      const auto slash = name.find('/');
      state_handles_.emplace_back(name.substr(0, slash), name.substr(slash + 1U), &values_[name]);
    }
    for (const auto & joint : kJoints) {
      values_[joint + "/status_word"] = kSwitchOnDisabled;
    }
    values_["steer_joint/mode_of_operation_display"] = 8.0;
    for (const auto & wheel : kWheels) {
      values_[wheel + "/mode_of_operation_display"] = 9.0;
    }
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    for (auto & handle : command_handles_) {
      loaned_commands.emplace_back(handle);
    }
    for (auto & handle : state_handles_) {
      loaned_states.emplace_back(handle);
    }
    controller_->assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
    ASSERT_EQ(
      controller_->on_activate(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
    for (int cycle = 0; cycle < 200 && Access::phase(*controller_) != Phase::kIdle; ++cycle) {
      update();
    }
    ASSERT_EQ(Access::phase(*controller_), Phase::kIdle);
  }

  // Enabled in VELOCITY_READY: every drive in operation enabled and in its startup mode.
  void enableDirectly()
  {
    for (const auto & joint : kJoints) {
      values_[joint + "/status_word"] = kOperationEnabled;
    }
    Access::setPhase(*controller_, Phase::kEnabled);
    Access::setControlMode(*controller_, ControlMode::kVelocityReady);
    Access::setOwner(*controller_, Owner::kNone);
  }

  void update()
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    now_ns_ += 1000000;
    controller_->update(rclcpp::Time(now_ns_), rclcpp::Duration::from_nanoseconds(1000000));
  }

  double wheelCommand(const std::string & wheel) { return values_[wheel + "/mode_of_operation"]; }

  std::unique_ptr<EnableManagerController> controller_;
  std::map<std::string, double> values_;
  std::vector<hardware_interface::CommandInterface> command_handles_;
  std::vector<hardware_interface::StateInterface> state_handles_;
  std::mutex update_mutex_;
  std::int64_t now_ns_{1000000000};
};

TEST_F(DriveModeFixture, ClaimsTheModeCommandOfEverySwitchedDrive)
{
  ASSERT_EQ(configure(), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    controller_->command_interface_configuration().names,
    (std::vector<std::string>{
      "steer_joint/control_word", "wheel_a_joint/control_word", "wheel_b_joint/control_word",
      "wheel_a_joint/mode_of_operation", "wheel_b_joint/mode_of_operation"}));
}

TEST_F(DriveModeFixture, RejectsSwitchedDrivesOutsideAVelocityModeGroup)
{
  EXPECT_EQ(
    configure({rclcpp::Parameter("trajectory_controller", std::string(""))}),
    controller_interface::CallbackReturn::ERROR);
  EXPECT_EQ(
    configure({rclcpp::Parameter("rolling_controller", std::string("rolling"))}),
    controller_interface::CallbackReturn::ERROR);
  // A drive whose startup mode is CSP has nothing to switch to.
  EXPECT_EQ(
    configure({rclcpp::Parameter("mode_switch_joints", std::vector<std::string>{"steer_joint"})}),
    controller_interface::CallbackReturn::ERROR);
  EXPECT_EQ(
    configure({rclcpp::Parameter("mode_service", std::string(""))}),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(DriveModeFixture, ControllerSetsMapToVelocityAndTrajectoryModes)
{
  ASSERT_EQ(configure(), controller_interface::CallbackReturn::SUCCESS);
  ListControllers::Response states;
  states.controller.resize(2U);
  states.controller[0].name = "swerve_controller";
  states.controller[1].name = "wheel_jtc";
  states.controller[0].state = "active";
  states.controller[1].state = "inactive";
  EXPECT_EQ(Access::detectControlMode(*controller_, states), ControlMode::kVelocityReady);
  states.controller[0].state = "inactive";
  states.controller[1].state = "active";
  EXPECT_EQ(Access::detectControlMode(*controller_, states), ControlMode::kFjtReady);
  states.controller[0].state = "active";
  EXPECT_EQ(Access::detectControlMode(*controller_, states), ControlMode::kRestartRequired);
}

TEST_F(DriveModeFixture, WritesCspOnlyWhileEnabledAndClearsTheRequestOtherwise)
{
  bringUpToIdle();
  update();
  EXPECT_EQ(wheelCommand("wheel_a_joint"), 9.0);
  enableDirectly();
  Access::requestCsp(*controller_, true);
  update();
  EXPECT_EQ(wheelCommand("wheel_a_joint"), 8.0);
  EXPECT_EQ(wheelCommand("wheel_b_joint"), 8.0);
  // Leaving the enabled phase returns the drives to the startup mode for the next enable.
  Access::setPhase(*controller_, Phase::kDisabling);
  update();
  EXPECT_EQ(wheelCommand("wheel_a_joint"), 9.0);
  EXPECT_FALSE(Access::cspRequested(*controller_));
}

TEST_F(DriveModeFixture, EnabledGroupAcceptsCspOnlyOnSwitchedDrives)
{
  bringUpToIdle();
  enableDirectly();
  for (const auto & wheel : kWheels) {
    values_[wheel + "/mode_of_operation_display"] = 8.0;
  }
  update();
  EXPECT_EQ(Access::phase(*controller_), Phase::kEnabled);
  values_["steer_joint/mode_of_operation_display"] = 9.0;
  update();
  EXPECT_EQ(Access::phase(*controller_), Phase::kEmergencyQuickStop);
}

// Fake controller_manager plus a realtime pump whose drives follow the mode command after a few
// cycles (or never, with confirm=false).
class DriveModeServiceFixture : public DriveModeFixture
{
public:
  void startPump(bool confirm)
  {
    manager_ = std::make_shared<rclcpp::Node>("fake_base_controller_manager");
    states_.controller.resize(2U);
    states_.controller[0].name = "swerve_controller";
    states_.controller[0].state = "active";
    states_.controller[1].name = "wheel_jtc";
    states_.controller[1].state = "inactive";
    list_ = manager_->create_service<ListControllers>(
      "/controller_manager/list_controllers",
      [this](
        const std::shared_ptr<ListControllers::Request>,
        std::shared_ptr<ListControllers::Response> response) {
        std::lock_guard<std::mutex> lock(states_mutex_);
        *response = states_;
      });
    switch_ = manager_->create_service<SwitchController>(
      "/controller_manager/switch_controller",
      [this](
        const std::shared_ptr<SwitchController::Request> request,
        std::shared_ptr<SwitchController::Response> response) {
        std::lock_guard<std::mutex> lock(states_mutex_);
        switch_requests_.push_back(*request);
        displays_at_switch_.push_back(display_a_.load());
        for (auto & controller : states_.controller) {
          for (const auto & name : request->activate_controllers) {
            controller.state = controller.name == name ? "active" : controller.state;
          }
          for (const auto & name : request->deactivate_controllers) {
            controller.state = controller.name == name ? "inactive" : controller.state;
          }
        }
        response->ok = true;
      });
    executor_ =
      std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 3U);
    executor_->add_node(controller_->get_node()->get_node_base_interface());
    executor_->add_node(manager_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    pump_thread_ = std::thread([this, confirm]() {
      auto jtc_state = std::make_shared<control_msgs::msg::JointTrajectoryControllerState>();
      jtc_state->joint_names = kJoints;
      jtc_state->output.positions.assign(kJoints.size(), 0.0);
      int lag = 0;
      while (pumping_.load()) {
        Access::handleJtcState(*controller_, jtc_state);
        update();
        const double command = wheelCommand("wheel_a_joint");
        const double display = values_["wheel_a_joint/mode_of_operation_display"];
        lag = command == display ? 0 : lag + 1;
        if (confirm && lag >= 3) {
          for (const auto & wheel : kWheels) {
            values_[wheel + "/mode_of_operation_display"] = command;
          }
        }
        display_a_.store(values_["wheel_a_joint/mode_of_operation_display"]);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  }

  std::shared_ptr<Access::ModeService::Response> request(std::uint8_t mode)
  {
    auto request = std::make_shared<Access::ModeService::Request>();
    request->target_mode.value = mode;
    auto response = std::make_shared<Access::ModeService::Response>();
    Access::handleSetMode(*controller_, request, response);
    return response;
  }

  void TearDown() override
  {
    pumping_.store(false);
    if (pump_thread_.joinable()) {
      pump_thread_.join();
    }
    if (executor_) {
      executor_->cancel();
      spin_thread_.join();
      executor_->remove_node(manager_);
      executor_->remove_node(controller_->get_node()->get_node_base_interface());
    }
    DriveModeFixture::TearDown();
  }

  std::shared_ptr<rclcpp::Node> manager_;
  rclcpp::Service<ListControllers>::SharedPtr list_;
  rclcpp::Service<SwitchController>::SharedPtr switch_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::thread pump_thread_;
  std::atomic_bool pumping_{true};
  std::mutex states_mutex_;
  ListControllers::Response states_;
  std::vector<SwitchController::Request> switch_requests_;
  std::vector<double> displays_at_switch_;
  std::atomic<double> display_a_{9.0};
};

TEST_F(DriveModeServiceFixture, SwitchesToCspAfterTheTrajectoryControllerHoldsAndBack)
{
  bringUpToIdle();
  enableDirectly();
  startPump(true);

  const auto to_trajectory = request(ModeMessage::FJT_READY);
  EXPECT_TRUE(to_trajectory->accepted) << to_trajectory->error.detail;
  EXPECT_EQ(to_trajectory->mode.value, ModeMessage::FJT_READY);
  EXPECT_EQ(display_a_.load(), 8.0);

  const auto to_velocity = request(ModeMessage::VELOCITY_READY);
  EXPECT_TRUE(to_velocity->accepted) << to_velocity->error.detail;
  EXPECT_EQ(to_velocity->mode.value, ModeMessage::VELOCITY_READY);
  EXPECT_EQ(display_a_.load(), 9.0);

  std::lock_guard<std::mutex> lock(states_mutex_);
  ASSERT_EQ(switch_requests_.size(), 2U);
  EXPECT_EQ(switch_requests_[0].activate_controllers, std::vector<std::string>{"wheel_jtc"});
  EXPECT_EQ(
    switch_requests_[0].deactivate_controllers, std::vector<std::string>{"swerve_controller"});
  // CSP only after the trajectory controller took over; CSV again before it hands back.
  EXPECT_EQ(displays_at_switch_[0], 9.0);
  EXPECT_EQ(displays_at_switch_[1], 9.0);
  EXPECT_EQ(
    switch_requests_[1].activate_controllers, std::vector<std::string>{"swerve_controller"});
}

TEST_F(DriveModeServiceFixture, EnableActivatesTheVelocityController)
{
  bringUpToIdle();
  startPump(true);
  {
    std::lock_guard<std::mutex> lock(states_mutex_);
    states_.controller[0].state = "inactive";
  }
  EXPECT_TRUE(Access::activateMotionControllers(*controller_));
  std::lock_guard<std::mutex> lock(states_mutex_);
  ASSERT_EQ(switch_requests_.size(), 1U);
  EXPECT_EQ(
    switch_requests_[0].activate_controllers, std::vector<std::string>{"swerve_controller"});
  EXPECT_EQ(states_.controller[1].state, "inactive");
}

TEST_F(DriveModeServiceFixture, UnconfirmedCspStopsTheGroup)
{
  bringUpToIdle();
  enableDirectly();
  startPump(false);
  const auto response = request(ModeMessage::FJT_READY);
  EXPECT_FALSE(response->accepted);
  EXPECT_EQ(response->error.code, robot_system_interfaces::msg::ErrorCode::TIMEOUT);
  EXPECT_EQ(response->error.detail, "drive_mode_not_confirmed");
  EXPECT_EQ(response->mode.value, ModeMessage::DISABLED);
}

TEST_F(DriveModeServiceFixture, RollingIsNotAModeOfAVelocityGroup)
{
  bringUpToIdle();
  enableDirectly();
  const auto response = request(ModeMessage::ROLLING_READY);
  EXPECT_FALSE(response->accepted);
  EXPECT_EQ(response->error.detail, "wrong_mode");
  EXPECT_EQ(response->mode.value, ModeMessage::VELOCITY_READY);
}

}  // namespace
}  // namespace ctrl_enable_manager
