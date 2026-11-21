// Copyright 2026 kkozia
// Licensed under the Apache License, Version 2.0
//
// Two motion groups under one enable state: the arms (trajectory <-> rolling) and the chassis
// (swerve <-> wheel trajectory, drive wheels CSV <-> CSP), plus one companion controller.

#include <gmock/gmock.h>

#include <algorithm>
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
  static void setOwner(Controller & c, Owner o) { c.owner_.store(o); }
  static std::size_t groupCount(const Controller & c) { return c.motion_group_count_; }
  static ControlMode mode(const Controller & c, std::size_t g)
  {
    return c.motion_groups_[g].current_mode.load();
  }
  static void storeEnabledModes(Controller & c) { c.storeEnabledGroupModes(); }
  static bool aborted(const Controller & c, std::size_t g)
  {
    return c.motion_groups_[g].abort_requested.load();
  }
  static std::string modeService(const Controller & c, std::size_t g)
  {
    return c.motion_groups_[g].mode_service->get_service_name();
  }
  static std::string standstillService(const Controller & c, std::size_t g)
  {
    return c.motion_groups_[g].standstill_service->get_service_name();
  }
  static bool disableRequested(const Controller & c) { return c.disable_request_.load(); }
  static bool enable(Controller & c)
  {
    return c.switchJtc(true) == Controller::SwitchResult::kSuccess;
  }
  static void handleSetMode(
    Controller & c, std::size_t g, std::shared_ptr<ModeService::Request> request,
    std::shared_ptr<ModeService::Response> response)
  {
    c.handleSetMode(c.motion_groups_[g], request, response);
  }
  static void handleJtcState(
    Controller & c, std::size_t g,
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr & message)
  {
    c.handleJtcState(c.motion_groups_[g], message);
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

constexpr std::size_t kArms = 0U;
constexpr std::size_t kBase = 1U;
constexpr double kOperationEnabled = 0x0027;
constexpr double kSwitchOnDisabled = 0x0040;
const std::vector<std::string> kArmJoints{"arm_a", "arm_b"};
const std::vector<std::string> kBaseJoints{"steer", "wheel_a", "wheel_b"};
const std::vector<std::string> kWheels{"wheel_a", "wheel_b"};
const std::vector<std::string> kAllJoints{"arm_a", "arm_b", "steer", "wheel_a", "wheel_b"};
const std::vector<std::string> kControllers{
  "arm_jtc", "arm_rolling", "swerve_controller", "wheel_jtc", "head_controller"};

std::vector<rclcpp::Parameter> twoGroupParameters()
{
  return {
    rclcpp::Parameter("managed_joints", kAllJoints),
    rclcpp::Parameter("enable_batch_joint_names", kAllJoints),
    rclcpp::Parameter("enable_batch_sizes", std::vector<std::int64_t>{3, 2}),
    rclcpp::Parameter("disable_terminal_policy", std::string("switch_on_disabled")),
    // Fixed startup modes with a rolling group: CSP arms, CSP steering, CSV drive wheels.
    rclcpp::Parameter("expected_modes", std::vector<std::int64_t>{8, 8, 8, 9, 9}),
    rclcpp::Parameter("companion_controllers", std::vector<std::string>{"head_controller"}),
    rclcpp::Parameter("motion_groups", std::vector<std::string>{"arms", "base"}),
    rclcpp::Parameter("arms.motion_controller", std::string("arm_jtc")),
    rclcpp::Parameter("arms.rolling_controller", std::string("arm_rolling")),
    rclcpp::Parameter("arms.mode_service", std::string("/rt/arms/set_control_mode")),
    rclcpp::Parameter("arms.motion_joints", kArmJoints),
    rclcpp::Parameter("arms.mode_switch_stable_velocity_thresholds", std::vector<double>(2U, 0.01)),
    rclcpp::Parameter("arms.mode_switch_takeover_tolerances", std::vector<double>(2U, 0.0087)),
    rclcpp::Parameter("base.motion_controller", std::string("swerve_controller")),
    rclcpp::Parameter("base.trajectory_controller", std::string("wheel_jtc")),
    rclcpp::Parameter("base.mode_service", std::string("/rt/base/set_control_mode")),
    rclcpp::Parameter("base.motion_joints", kBaseJoints),
    rclcpp::Parameter("base.mode_switch_joints", kWheels),
    rclcpp::Parameter(
      "base.mode_switch_stable_velocity_thresholds", std::vector<double>{0.01, 0.1, 0.1}),
    rclcpp::Parameter("base.mode_switch_takeover_tolerances", std::vector<double>(3U, 0.0087)),
    rclcpp::Parameter("controller_switch_timeout", 0.5),
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

class MotionGroupsFixture : public ::testing::Test
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
    if (controller_) {
      controller_->release_interfaces();
      controller_.reset();
    }
  }

  controller_interface::CallbackReturn configure(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    auto parameters = twoGroupParameters();
    for (const auto & parameter : overrides) {
      replace(parameters, parameter);
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    controller_ = std::make_unique<EnableManagerController>();
    EXPECT_EQ(
      controller_->init("motion_groups_test", "", options), controller_interface::return_type::OK);
    return controller_->on_configure(rclcpp_lifecycle::State());
  }

  void bringUpToIdle()
  {
    ASSERT_EQ(configure(twoGroupOverrides_), controller_interface::CallbackReturn::SUCCESS);
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
    for (const auto & joint : kAllJoints) {
      values_[joint + "/status_word"] = kSwitchOnDisabled;
    }
    for (const auto & joint : {"arm_a", "arm_b", "steer"}) {
      if (values_.count(std::string(joint) + "/mode_of_operation_display") != 0U) {
        values_[std::string(joint) + "/mode_of_operation_display"] = 8.0;
      }
    }
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

  void enableDirectly()
  {
    for (const auto & joint : kAllJoints) {
      values_[joint + "/status_word"] = kOperationEnabled;
    }
    Access::setPhase(*controller_, Phase::kEnabled);
    Access::storeEnabledModes(*controller_);
    Access::setOwner(*controller_, Owner::kNone);
    std::lock_guard<std::mutex> lock(states_mutex_);
    for (auto & controller : states_.controller) {
      controller.state = controller.name == "arm_jtc" || controller.name == "swerve_controller" ||
                             controller.name == "head_controller"
                           ? "active"
                           : "inactive";
    }
  }

  void update()
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    now_ns_ += 1000000;
    controller_->update(rclcpp::Time(now_ns_), rclcpp::Duration::from_nanoseconds(1000000));
  }

  // Fake controller_manager; a realtime pump that publishes both trajectory controllers' state,
  // moves arm_a while move_arm_ is set, and lets the drive wheels follow their mode command.
  void startManagerAndPump(bool confirm_drive_modes = true)
  {
    manager_ = std::make_shared<rclcpp::Node>("fake_controller_manager");
    for (const auto & name : kControllers) {
      controller_manager_msgs::msg::ControllerState state;
      state.name = name;
      state.state = "inactive";
      states_.controller.push_back(state);
    }
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
    pump_thread_ = std::thread([this, confirm_drive_modes]() {
      int lag = 0;
      while (pumping_.load()) {
        {
          std::lock_guard<std::mutex> lock(update_mutex_);
          if (move_arm_.load()) {
            values_["arm_a/position"] += 0.001;
          }
        }
        publishTrajectoryState(kArms, kArmJoints);
        publishTrajectoryState(kBase, kBaseJoints);
        update();
        const double command = values_["wheel_a/mode_of_operation"];
        lag = command == values_["wheel_a/mode_of_operation_display"] ? 0 : lag + 1;
        if (confirm_drive_modes && lag >= 3) {
          for (const auto & wheel : kWheels) {
            values_[wheel + "/mode_of_operation_display"] = command;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  }

  void publishTrajectoryState(std::size_t group, const std::vector<std::string> & joints)
  {
    auto state = std::make_shared<control_msgs::msg::JointTrajectoryControllerState>();
    state->joint_names = joints;
    {
      std::lock_guard<std::mutex> lock(update_mutex_);
      for (const auto & joint : joints) {
        state->output.positions.push_back(values_[joint + "/position"]);
      }
    }
    Access::handleJtcState(*controller_, group, state);
  }

  std::shared_ptr<Access::ModeService::Response> request(std::size_t group, std::uint8_t mode)
  {
    auto request = std::make_shared<Access::ModeService::Request>();
    request->target_mode.value = mode;
    auto response = std::make_shared<Access::ModeService::Response>();
    Access::handleSetMode(*controller_, group, request, response);
    return response;
  }

  std::string controllerState(const std::string & name)
  {
    std::lock_guard<std::mutex> lock(states_mutex_);
    for (const auto & controller : states_.controller) {
      if (controller.name == name) {
        return controller.state;
      }
    }
    return "missing";
  }

  std::unique_ptr<EnableManagerController> controller_;
  std::vector<rclcpp::Parameter> twoGroupOverrides_;
  std::map<std::string, double> values_;
  std::vector<hardware_interface::CommandInterface> command_handles_;
  std::vector<hardware_interface::StateInterface> state_handles_;
  std::mutex update_mutex_;
  std::int64_t now_ns_{1000000000};
  std::shared_ptr<rclcpp::Node> manager_;
  rclcpp::Service<ListControllers>::SharedPtr list_;
  rclcpp::Service<SwitchController>::SharedPtr switch_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::thread pump_thread_;
  std::atomic_bool pumping_{true};
  std::atomic_bool move_arm_{false};
  std::mutex states_mutex_;
  ListControllers::Response states_;
  std::vector<SwitchController::Request> switch_requests_;
};

TEST_F(MotionGroupsFixture, EachGroupHasItsInterfacesAndServices)
{
  ASSERT_EQ(configure(), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(Access::groupCount(*controller_), 2U);
  const auto states = controller_->state_interface_configuration().names;
  const std::vector<std::string> motion(states.begin() + 5, states.begin() + 11);
  EXPECT_EQ(
    motion, (std::vector<std::string>{
              "arm_a/position", "arm_b/position", "steer/position", "wheel_a/position",
              "wheel_b/position", "ethercat_domain/process_data_age_ms"}));
  const auto commands = controller_->command_interface_configuration().names;
  EXPECT_EQ(
    std::vector<std::string>(commands.end() - 2, commands.end()),
    (std::vector<std::string>{"wheel_a/mode_of_operation", "wheel_b/mode_of_operation"}));
  EXPECT_EQ(Access::modeService(*controller_, kArms), "/rt/arms/set_control_mode");
  EXPECT_EQ(Access::modeService(*controller_, kBase), "/rt/base/set_control_mode");
  EXPECT_EQ(
    Access::standstillService(*controller_, kArms), "/motion_groups_test/arms/confirm_standstill");
  EXPECT_EQ(
    Access::standstillService(*controller_, kBase), "/motion_groups_test/base/confirm_standstill");
}

TEST_F(MotionGroupsFixture, RejectsOverlappingOrAmbiguousGroups)
{
  const std::vector<std::vector<rclcpp::Parameter>> invalid{
    {rclcpp::Parameter(
      "base.motion_joints", std::vector<std::string>{"arm_a", "steer", "wheel_a", "wheel_b"})},
    {rclcpp::Parameter("base.motion_controller", std::string("arm_jtc"))},
    {rclcpp::Parameter("base.mode_service", std::string("/rt/arms/set_control_mode"))},
    {rclcpp::Parameter("base.trajectory_controller", std::string("")),
     rclcpp::Parameter("base.rolling_controller", std::string("base_rolling")),
     rclcpp::Parameter("base.mode_switch_joints", std::vector<std::string>{})},
    {rclcpp::Parameter("motion_groups", std::vector<std::string>{"arms", "arms"})},
    {rclcpp::Parameter("motion_groups", std::vector<std::string>{"arms", "ba.se"})},
    {rclcpp::Parameter("motion_groups", std::vector<std::string>{"a", "b", "c", "d", "e"})},
  };
  for (const auto & overrides : invalid) {
    SCOPED_TRACE(overrides.front().get_name());
    EXPECT_EQ(configure(overrides), controller_interface::CallbackReturn::ERROR);
    controller_.reset();
  }
}

TEST_F(MotionGroupsFixture, EnableActivatesEveryDefaultControllerAndTheCompanions)
{
  bringUpToIdle();
  startManagerAndPump();
  ASSERT_TRUE(Access::enable(*controller_));
  std::lock_guard<std::mutex> lock(states_mutex_);
  ASSERT_EQ(switch_requests_.size(), 1U);
  EXPECT_THAT(
    switch_requests_[0].activate_controllers,
    ::testing::UnorderedElementsAre("arm_jtc", "swerve_controller", "head_controller"));
  EXPECT_TRUE(switch_requests_[0].deactivate_controllers.empty());
}

TEST_F(MotionGroupsFixture, SwitchingOneGroupLeavesTheOtherAlone)
{
  bringUpToIdle();
  startManagerAndPump();
  enableDirectly();
  EXPECT_EQ(Access::mode(*controller_, kArms), ControlMode::kFjtReady);
  EXPECT_EQ(Access::mode(*controller_, kBase), ControlMode::kVelocityReady);

  // The chassis switches while an arm is moving: only the chassis must stand still.
  move_arm_.store(true);
  const auto base = request(kBase, ModeMessage::FJT_READY);
  move_arm_.store(false);
  EXPECT_TRUE(base->accepted) << base->error.detail;
  EXPECT_EQ(Access::mode(*controller_, kBase), ControlMode::kFjtReady);
  EXPECT_EQ(Access::mode(*controller_, kArms), ControlMode::kFjtReady);
  EXPECT_EQ(values_["wheel_a/mode_of_operation_display"], 8.0);
  {
    std::lock_guard<std::mutex> lock(states_mutex_);
    ASSERT_EQ(switch_requests_.size(), 1U);
    EXPECT_EQ(switch_requests_[0].activate_controllers, std::vector<std::string>{"wheel_jtc"});
    EXPECT_EQ(
      switch_requests_[0].deactivate_controllers, std::vector<std::string>{"swerve_controller"});
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const auto arms = request(kArms, ModeMessage::ROLLING_READY);
  EXPECT_TRUE(arms->accepted) << arms->error.detail;
  EXPECT_EQ(Access::mode(*controller_, kArms), ControlMode::kRollingReady);
  EXPECT_EQ(Access::mode(*controller_, kBase), ControlMode::kFjtReady);
  EXPECT_EQ(controllerState("wheel_jtc"), "active");
  EXPECT_EQ(controllerState("head_controller"), "active");
  std::lock_guard<std::mutex> lock(states_mutex_);
  ASSERT_EQ(switch_requests_.size(), 2U);
  EXPECT_EQ(switch_requests_[1].activate_controllers, std::vector<std::string>{"arm_rolling"});
  EXPECT_EQ(switch_requests_[1].deactivate_controllers, std::vector<std::string>{"arm_jtc"});
}

TEST_F(MotionGroupsFixture, ASwitchInProgressMakesTheOtherGroupBusy)
{
  bringUpToIdle();
  enableDirectly();
  Access::setOwner(*controller_, Owner::kMode);
  const auto base = request(kBase, ModeMessage::FJT_READY);
  EXPECT_FALSE(base->accepted);
  EXPECT_EQ(base->error.detail, "busy");
  EXPECT_EQ(Access::mode(*controller_, kBase), ControlMode::kVelocityReady);
}

TEST_F(MotionGroupsFixture, AFailedSwitchInOneGroupDisablesTheManager)
{
  bringUpToIdle();
  startManagerAndPump(false);
  enableDirectly();
  const auto base = request(kBase, ModeMessage::FJT_READY);
  EXPECT_FALSE(base->accepted);
  EXPECT_EQ(base->error.detail, "drive_mode_not_confirmed");
  EXPECT_TRUE(Access::disableRequested(*controller_));
}

TEST_F(MotionGroupsFixture, DeactivationAbortsEveryGroup)
{
  bringUpToIdle();
  controller_->on_deactivate(rclcpp_lifecycle::State());
  EXPECT_TRUE(Access::aborted(*controller_, kArms));
  EXPECT_TRUE(Access::aborted(*controller_, kBase));
  EXPECT_EQ(Access::mode(*controller_, kArms), ControlMode::kDisabled);
  EXPECT_EQ(Access::mode(*controller_, kBase), ControlMode::kDisabled);
}

TEST_F(MotionGroupsFixture, OnlyTheSwitchedDrivesMayRunInCsp)
{
  bringUpToIdle();
  enableDirectly();
  for (const auto & wheel : kWheels) {
    values_[wheel + "/mode_of_operation_display"] = 8.0;
  }
  update();
  EXPECT_EQ(Access::phase(*controller_), Phase::kEnabled);
  values_["arm_b/mode_of_operation_display"] = 9.0;
  update();
  EXPECT_EQ(Access::phase(*controller_), Phase::kEmergencyQuickStop);
}

TEST_F(MotionGroupsFixture, UnconfirmedJointsClaimNoModeDisplay)
{
  // Arm drives without a mode display: 0 leaves their startup mode unconfirmed.
  ASSERT_EQ(
    configure({rclcpp::Parameter("expected_modes", std::vector<std::int64_t>{0, 0, 8, 9, 9})}),
    controller_interface::CallbackReturn::SUCCESS);
  const auto states = controller_->state_interface_configuration().names;
  for (const auto & joint : kArmJoints) {
    EXPECT_EQ(std::count(states.begin(), states.end(), joint + "/mode_of_operation_display"), 0);
  }
  for (const auto & joint : kBaseJoints) {
    EXPECT_EQ(std::count(states.begin(), states.end(), joint + "/mode_of_operation_display"), 1);
  }
}

TEST_F(MotionGroupsFixture, AnUnconfirmedJointNeverTripsTheModeCheck)
{
  ASSERT_EQ(
    configure({rclcpp::Parameter("expected_modes", std::vector<std::int64_t>{0, 0, 8, 9, 9})}),
    controller_interface::CallbackReturn::SUCCESS);
  controller_.reset();
  twoGroupOverrides_ = {
    rclcpp::Parameter("expected_modes", std::vector<std::int64_t>{0, 0, 8, 9, 9})};
  bringUpToIdle();
  enableDirectly();
  update();
  EXPECT_EQ(Access::phase(*controller_), Phase::kEnabled);
  // A confirmed joint still stops the manager when its mode is wrong.
  values_["steer/mode_of_operation_display"] = 9.0;
  update();
  EXPECT_EQ(Access::phase(*controller_), Phase::kEmergencyQuickStop);
}

TEST_F(MotionGroupsFixture, ASwitchedDriveNeedsAConfirmedStartupMode)
{
  EXPECT_EQ(
    configure({rclcpp::Parameter("expected_modes", std::vector<std::int64_t>{8, 8, 8, 0, 9})}),
    controller_interface::CallbackReturn::ERROR);
}

}  // namespace
}  // namespace ctrl_enable_manager
