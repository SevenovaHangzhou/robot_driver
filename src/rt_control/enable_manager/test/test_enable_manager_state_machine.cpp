// Copyright 2026 kkozia
// Licensed under the Apache License, Version 2.0
//
// Table-driven unit tests for the enable_manager ros2_control controller.
//
// controller_manager scope decision:
//   switchJtc() talks to /controller_manager/{switch_controller,list_controllers}.
//   In unit scope no controller_manager exists and switchJtc() short-circuits to
//   SwitchResult::kAmbiguous on its wait_for_service(0ms) probe. Instead of
//   standing up a fake controller_manager, these tests drive the RT state machine
//   through update() directly and assert on control words / phase / result slots.
//   The JTC boundary is covered as the kAmbiguous branch (test f). Service entry
//   points that do NOT need controller_manager (already_*, busy, inactive) are
//   invoked through the real callbacks.

#include <gmock/gmock.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "enable_manager/enable_manager_controller.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/joint_control_mode.hpp"
#include "robot_rt_control_interfaces/msg/rolling_service_result.hpp"
#include "rt_control_interfaces/msg/joint_control_mode_result.hpp"
#include "rt_control_interfaces/srv/rt_enable.hpp"

namespace enable_manager
{

// Access shim, enabled by the single friend declaration in the production header.
class EnableManagerTestAccess
{
public:
  using Controller = EnableManagerController;
  using Phase = EnableManagerController::Phase;
  using Owner = EnableManagerController::Owner;
  using Stage = EnableManagerController::Stage;
  using DriveState = EnableManagerController::DriveState;
  using SwitchResult = EnableManagerController::SwitchResult;
  using ControlMode = EnableManagerController::ControlMode;
  using ModeSwitchState = EnableManagerController::ModeSwitchState;

  static constexpr std::size_t kAxisCount = EnableManagerController::kAxisCount;
  static constexpr std::size_t kBatchCount = EnableManagerController::kBatchCount;

  static const std::array<const char *, kAxisCount> & jointNames()
  {
    return EnableManagerController::kJointNames;
  }
  static const std::array<std::array<std::int8_t, 3>, kBatchCount> & batches()
  {
    return EnableManagerController::kEnableBatches;
  }
  static const std::array<std::uint8_t, kBatchCount> & batchSizes()
  {
    return EnableManagerController::kBatchSizes;
  }

  static Phase phase(const Controller & c) {return c.phase_.load();}
  static Owner owner(const Controller & c) {return c.owner_.load();}
  static std::size_t currentBatch(const Controller & c) {return c.current_batch_;}
  static bool restartRequired(const Controller & c) {return c.restart_required_.load();}
  static void setPhase(Controller & c, Phase p) {c.phase_.store(p);}
  static void setOwner(Controller & c, Owner o) {c.owner_.store(o);}
  static void setControlMode(Controller & c, ControlMode mode)
  {
    c.current_control_mode_.store(mode);
  }

  static DriveState decode(std::uint16_t sw) {return EnableManagerController::decodeState(sw);}
  static const char * stageName(Stage s) {return EnableManagerController::stageName(s);}
  static const char * phaseName(Phase p) {return EnableManagerController::phaseName(p);}
  static SwitchResult switchJtc(Controller & c, bool a) {return c.switchJtc(a);}
  static SwitchResult buildSwitchRequest(
    const Controller & c,
    const controller_manager_msgs::srv::ListControllers::Response & states,
    bool activate_default,
    controller_manager_msgs::srv::SwitchController::Request & request)
  {
    return c.buildMotionControllerSwitchRequest(states, activate_default, request);
  }
  static bool verifyRegistryState(
    const Controller & c,
    const controller_manager_msgs::srv::ListControllers::Response & states,
    bool default_active)
  {
    return c.registeredMotionControllerStateMatches(states, default_active);
  }
  static ControlMode detectControlMode(
    const Controller & c,
    const controller_manager_msgs::srv::ListControllers::Response & states)
  {
    return c.detectControlMode(states);
  }
  static ModeSwitchState classifyModeSwitchState(
    const Controller & c,
    const controller_manager_msgs::srv::ListControllers::Response & states,
    ControlMode source, ControlMode target)
  {
    return c.classifyModeSwitchState(states, source, target);
  }
  static bool stableInterval(
    const Controller & c,
    const std::array<double, kAxisCount> & previous,
    const std::array<double, kAxisCount> & current,
    std::uint64_t previous_time_ns, std::uint64_t current_time_ns)
  {
    EnableManagerController::ActualSample previous_sample;
    previous_sample.positions = previous;
    previous_sample.steady_time_ns = previous_time_ns;
    EnableManagerController::ActualSample current_sample;
    current_sample.positions = current;
    current_sample.steady_time_ns = current_time_ns;
    return c.stableActualInterval(previous_sample, current_sample);
  }
  static bool sourceWithinTakeover(
    const Controller & c,
    const std::array<double, kAxisCount> & source,
    const std::array<double, kAxisCount> & actual)
  {
    return c.sourceWithinTakeoverTolerance(source, actual);
  }

  using ResultSlot = EnableManagerController::ResultSlot;

  // Direct RT-loop drivers. The service callbacks set these same flags; setting
  // them here lets a test exercise the state machine without the
  // controller_manager round trip the callbacks require.
  static void setEnableRequest(Controller & c, bool v) {c.enable_request_.store(v);}
  static void setDisableRequest(Controller & c, bool v) {c.disable_request_.store(v);}
  static void setResetRequest(Controller & c, bool v) {c.reset_request_.store(v);}
  static bool enableHardwareReady(const Controller & c)
  {
    return c.enable_hardware_ready_.load();
  }

  static ResultSlot & enableResult(Controller & c) {return c.enable_result_;}
  static ResultSlot & disableResult(Controller & c) {return c.disable_result_;}
  static ResultSlot & resetResult(Controller & c) {return c.reset_result_;}

  // Converts a result slot into the exact response the service would return.
  static void fillResponse(
    const Controller & c, const ResultSlot & slot,
    rt_control_interfaces::srv::RtEnable::Response & response)
  {
    c.fillResponse(slot, response);
  }

  static Stage slotStage(const ResultSlot & s) {return s.stage.load();}
  static bool slotReady(const ResultSlot & s) {return s.ready.load();}
  static bool slotOk(const ResultSlot & s) {return s.ok.load();}
  static std::int8_t slotFailedBatch(const ResultSlot & s) {return s.failed_batch.load();}
  static std::int8_t slotFailedJoint(const ResultSlot & s) {return s.failed_joint.load();}
  static std::uint16_t slotStatusWord(const ResultSlot & s) {return s.status_word.load();}

  using Req = std::shared_ptr<rt_control_interfaces::srv::RtEnable::Request>;
  using Res = std::shared_ptr<rt_control_interfaces::srv::RtEnable::Response>;
  static void handleEnable(Controller & c, Req q, Res s) {c.handleEnable(q, s);}
  static void handleDisable(Controller & c, Req q, Res s) {c.handleDisable(q, s);}
  static void handleResetFault(Controller & c, Req q, Res s) {c.handleResetFault(q, s);}
  using ModeReq = std::shared_ptr<EnableManagerController::ModeService::Request>;
  using ModeRes = std::shared_ptr<EnableManagerController::ModeService::Response>;
  static void handleSetMode(Controller & c, ModeReq q, ModeRes s)
  {
    c.handleSetMode(q, s);
  }
  static void handleJtcState(
    Controller & c,
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr & message)
  {
    c.handleJtcState(message);
  }
  static void handleRollingState(
    Controller & c,
    const robot_rt_control_interfaces::msg::RollingJointControlState::SharedPtr & message)
  {
    c.handleRollingState(message);
  }
  static ControlMode controlMode(const Controller & c)
  {
    return c.current_control_mode_.load();
  }
  static bool modeAbortRequested(const Controller & c)
  {
    return c.mode_abort_requested_.load();
  }
  static bool switchInProgress(const Controller & c)
  {
    return c.switch_in_progress_.load();
  }
  static void setSwitchInProgress(Controller & c, bool value)
  {
    c.switch_in_progress_.store(value);
  }
  static void requestEmergencyControllerStop(Controller & c)
  {
    c.jtc_deactivation_required_.store(true);
    c.emergency_jtc_deactivate_request_.store(true);
  }
  static bool emergencyControllerStopRequested(const Controller & c)
  {
    return c.emergency_jtc_deactivate_request_.load();
  }
  static bool waitForControllerManagerServices(
    Controller & c, std::chrono::milliseconds timeout)
  {
    return c.list_client_->wait_for_service(timeout) &&
           c.switch_client_->wait_for_service(timeout);
  }
  static void handleNonRtFaultStop(Controller & c)
  {
    c.handleNonRtFaultStop();
  }
  static void startEmergency(
    Controller & c, Stage stage, std::int8_t joint, std::uint16_t status,
    std::int64_t now_ns)
  {
    c.startEmergency(stage, joint, status, now_ns);
  }
};

namespace
{

using Access = EnableManagerTestAccess;
using Phase = Access::Phase;
using Owner = Access::Owner;
using Stage = Access::Stage;
using DriveState = Access::DriveState;
using SwitchResult = Access::SwitchResult;
using ControlMode = Access::ControlMode;
using ModeSwitchState = Access::ModeSwitchState;
using RtEnable = rt_control_interfaces::srv::RtEnable;
using ModeService = robot_rt_control_interfaces::srv::SetJointControlMode;
using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
using ModeResult = rt_control_interfaces::msg::JointControlModeResult;

// CiA402 status words that decodeState() maps to each DriveState.
constexpr std::uint16_t kSwNotReady = 0x0000U;
constexpr std::uint16_t kSwSwitchOnDisabled = 0x0040U;
constexpr std::uint16_t kSwReadyToSwitchOn = 0x0021U;
constexpr std::uint16_t kSwSwitchedOn = 0x0023U;
constexpr std::uint16_t kSwOperationEnabled = 0x0027U;
constexpr std::uint16_t kSwQuickStopActive = 0x0007U;
constexpr std::uint16_t kSwFault = 0x0008U;

// CiA402 control words the controller writes.
constexpr std::uint16_t kCwZero = 0x0000U;
constexpr std::uint16_t kCwQuickStop = 0x0002U;
constexpr std::uint16_t kCwShutdown = 0x0006U;
constexpr std::uint16_t kCwSwitchOn = 0x0007U;
constexpr std::uint16_t kCwEnableOperation = 0x000FU;
constexpr std::uint16_t kCwFaultReset = 0x0080U;

constexpr std::size_t kAxes = Access::kAxisCount;
constexpr std::size_t kBatches = Access::kBatchCount;

// Timing parameters used by every fixture instance. Small values keep the
// tests fast while remaining well above the 1 ms simulated control period.
constexpr double kBatchTimeout = 0.20;
constexpr double kDisableStageTimeout = 0.20;
constexpr double kInterBatchDelay = 0.02;
constexpr double kFaultResetTimeout = 0.20;
constexpr double kControllerSwitchTimeout = 0.05;
constexpr int kServiceResultTimeoutMs = 300;

class EnableManagerFixture : public ::testing::Test
{
public:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    controller_ = std::make_unique<EnableManagerController>();
    status_buffer_.fill(static_cast<double>(kSwSwitchOnDisabled));
    position_buffer_.fill(0.0);
    feedback_age_ms_ = 0.0;
    command_buffer_.fill(0.0);
  }

  void TearDown() override
  {
    if (controller_ != nullptr) {
      controller_->release_interfaces();
      controller_.reset();
    }
    command_handles_.clear();
    state_handles_.clear();
  }

  // Runs init/configure and assigns loaned interfaces backed by buffers this
  // fixture owns. Stops short of on_activate so tests can inspect that step.
  void initAndConfigure(const std::string & name = "enable_manager_test")
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {
          rclcpp::Parameter("batch_timeout", kBatchTimeout),
          rclcpp::Parameter("disable_stage_timeout", kDisableStageTimeout),
          rclcpp::Parameter("inter_batch_delay", kInterBatchDelay),
          rclcpp::Parameter("fault_reset_timeout", kFaultResetTimeout),
          rclcpp::Parameter("controller_switch_timeout", kControllerSwitchTimeout),
          rclcpp::Parameter("service_result_timeout_ms", kServiceResultTimeoutMs),
          rclcpp::Parameter(
            "motion_controller_names",
            std::vector<std::string>{"whole_body_jtc", "rolling_trajectory_controller"}),
          rclcpp::Parameter("default_motion_controller", std::string("whole_body_jtc")),
          rclcpp::Parameter("update_rate", 1000),
        });
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    ASSERT_EQ(
      controller_->init(name, "", options), controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
    assignInterfaces();
  }

  void assignInterfaces()
  {
    command_handles_.clear();
    state_handles_.clear();
    command_handles_.reserve(kAxes);
    state_handles_.reserve(kAxes * 2U + 1U);

    // The controller claims interfaces in kJointNames order, so build in that order.
    for (std::size_t axis = 0; axis < kAxes; ++axis) {
      command_handles_.emplace_back(
        Access::jointNames()[axis], "control_word", &command_buffer_[axis]);
      state_handles_.emplace_back(
        Access::jointNames()[axis], "status_word", &status_buffer_[axis]);
    }
    for (std::size_t axis = 0; axis < kAxes; ++axis) {
      state_handles_.emplace_back(
        Access::jointNames()[axis], "position", &position_buffer_[axis]);
    }
    state_handles_.emplace_back(
      "ethercat_domain", "process_data_age_ms", &feedback_age_ms_);

    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    loaned_commands.reserve(kAxes);
    loaned_states.reserve(kAxes * 2U + 1U);
    for (std::size_t axis = 0; axis < kAxes; ++axis) {
      loaned_commands.emplace_back(command_handles_[axis]);
    }
    for (auto & state_handle : state_handles_) {
      loaned_states.emplace_back(state_handle);
    }
    controller_->assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
  }

  void activate()
  {
    ASSERT_EQ(
      controller_->on_activate(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
  }

  // init + configure + activate, then walk the startup sanitize phase to kIdle.
  void bringUpToIdle()
  {
    initAndConfigure();
    activate();
    ASSERT_EQ(Access::phase(*controller_), Phase::kStartupSanitizing);
    setAllStatus(kSwSwitchOnDisabled);
    ASSERT_TRUE(spinUntilPhase(Phase::kIdle, 200)) << "startup sanitize did not reach IDLE";
    ASSERT_EQ(Access::owner(*controller_), Owner::kNone);
  }

  // ---- simulated clock -------------------------------------------------
  // update() only reads time.nanoseconds(), so the tests advance a virtual
  // clock instead of sleeping. Timeout paths become exact and instant.
  void advance(double seconds)
  {
    now_ns_ += static_cast<std::int64_t>(seconds * 1e9);
  }

  controller_interface::return_type callUpdate(double dt = 0.001)
  {
    advance(dt);
    return controller_->update(
      rclcpp::Time(now_ns_, RCL_ROS_TIME),
      rclcpp::Duration::from_seconds(dt));
  }

  void updateTimes(std::size_t count, double dt = 0.001)
  {
    for (std::size_t i = 0; i < count; ++i) {
      ASSERT_EQ(callUpdate(dt), controller_interface::return_type::OK);
    }
  }

  bool spinUntilPhase(Phase target, std::size_t max_iterations, double dt = 0.001)
  {
    for (std::size_t i = 0; i < max_iterations; ++i) {
      if (Access::phase(*controller_) == target) {
        return true;
      }
      if (callUpdate(dt) != controller_interface::return_type::OK) {
        return false;
      }
    }
    return Access::phase(*controller_) == target;
  }

  // ---- hardware simulation --------------------------------------------
  void setStatus(std::size_t axis, std::uint16_t status_word)
  {
    status_buffer_[axis] = static_cast<double>(status_word);
  }

  void setAllStatus(std::uint16_t status_word)
  {
    status_buffer_.fill(static_cast<double>(status_word));
  }

  std::uint16_t status(std::size_t axis) const
  {
    return static_cast<std::uint16_t>(status_buffer_[axis]);
  }

  std::uint16_t command(std::size_t axis) const
  {
    return static_cast<std::uint16_t>(command_buffer_[axis]);
  }

  // Mimics a healthy CiA402 drive: the status word follows the last control word.
  void stepDriveResponses()
  {
    for (std::size_t axis = 0; axis < kAxes; ++axis) {
      stepDriveResponse(axis);
    }
  }

  void stepDriveResponse(std::size_t axis)
  {
    const std::uint16_t cw = command(axis);
    const DriveState state = Access::decode(status(axis));
    if (state == DriveState::kFault || state == DriveState::kFaultReactionActive) {
      // Only a rising 0x0080 clears a fault; handled explicitly by the tests.
      return;
    }
    switch (cw) {
      case kCwShutdown:
        setStatus(axis, kSwReadyToSwitchOn);
        break;
      case kCwSwitchOn:
        setStatus(axis, kSwSwitchedOn);
        break;
      case kCwEnableOperation:
        setStatus(axis, kSwOperationEnabled);
        break;
      case kCwQuickStop:
        setStatus(axis, kSwQuickStopActive);
        break;
      case kCwZero:
      default:
        setStatus(axis, kSwSwitchOnDisabled);
        break;
    }
  }

  // Same as stepDriveResponses() but leaves the flagged axes frozen, which is
  // how the tests simulate a drive that never reaches its target state.
  void stepDriveResponses(const std::array<bool, kAxes> & stalled)
  {
    for (std::size_t axis = 0; axis < kAxes; ++axis) {
      if (!stalled[axis]) {
        stepDriveResponse(axis);
      }
    }
  }

  // ---- service invocation ---------------------------------------------
  // The three service callbacks block on the RT loop, so they must run on
  // another thread while the test thread keeps calling update().
  enum class ServiceKind {kEnable, kDisable, kReset};

  struct PendingCall
  {
    std::shared_ptr<RtEnable::Response> response;
    std::thread worker;
    std::atomic_bool done{false};
  };

  std::shared_ptr<PendingCall> callAsync(ServiceKind kind)
  {
    auto pending = std::make_shared<PendingCall>();
    pending->response = std::make_shared<RtEnable::Response>();
    auto request = std::make_shared<RtEnable::Request>();
    EnableManagerController * controller = controller_.get();
    pending->worker = std::thread(
      [kind, controller, pending, request]() {
        switch (kind) {
          case ServiceKind::kEnable:
            Access::handleEnable(*controller, request, pending->response);
            break;
          case ServiceKind::kDisable:
            Access::handleDisable(*controller, request, pending->response);
            break;
          case ServiceKind::kReset:
            Access::handleResetFault(*controller, request, pending->response);
            break;
        }
        pending->done.store(true);
      });
    return pending;
  }

  // Drives update() (with the drive simulation) until the pending call returns.
  // Returns false if it never completed within the iteration budget.
  bool pumpUntilDone(
    const std::shared_ptr<PendingCall> & pending, std::size_t max_iterations,
    const std::array<bool, kAxes> & stalled, double dt = 0.001)
  {
    for (std::size_t i = 0; i < max_iterations && !pending->done.load(); ++i) {
      if (callUpdate(dt) != controller_interface::return_type::OK) {
        return false;
      }
      stepDriveResponses(stalled);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return pending->done.load();
  }

  bool pumpUntilDone(
    const std::shared_ptr<PendingCall> & pending,
    std::size_t max_iterations = 4000)
  {
    const std::array<bool, kAxes> none{};
    return pumpUntilDone(pending, max_iterations, none);
  }

  static void join(const std::shared_ptr<PendingCall> & pending)
  {
    if (pending->worker.joinable()) {
      pending->worker.join();
    }
  }

  std::unique_ptr<EnableManagerController> controller_;
  std::array<double, kAxes> command_buffer_{};
  std::array<double, kAxes> status_buffer_{};
  std::array<double, kAxes> position_buffer_{};
  double feedback_age_ms_{0.0};
  std::vector<hardware_interface::CommandInterface> command_handles_;
  std::vector<hardware_interface::StateInterface> state_handles_;
  std::int64_t now_ns_{1000000000};
};

// ---------------------------------------------------------------------------
// Fixture sanity / interface contract
// ---------------------------------------------------------------------------

TEST_F(EnableManagerFixture, ClaimsModeAdmissionStateInterfaces)
{
  initAndConfigure();

  const auto command_config = controller_->command_interface_configuration();
  const auto state_config = controller_->state_interface_configuration();

  ASSERT_EQ(command_config.names.size(), kAxes);
  ASSERT_EQ(state_config.names.size(), kAxes * 2U + 1U);
  EXPECT_EQ(
    command_config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    EXPECT_EQ(
      command_config.names[axis], std::string(Access::jointNames()[axis]) + "/control_word");
    EXPECT_EQ(
      state_config.names[axis], std::string(Access::jointNames()[axis]) + "/status_word");
    EXPECT_EQ(
      state_config.names[kAxes + axis],
      std::string(Access::jointNames()[axis]) + "/position");
  }
  EXPECT_EQ(
    state_config.names[kAxes * 2U], "ethercat_domain/process_data_age_ms");
}

TEST_F(EnableManagerFixture, BatchTableCoversAllFourteenAxesExactlyOnce)
{
  ASSERT_EQ(kBatches, 5U);
  const std::array<std::uint8_t, kBatches> expected_sizes{3U, 3U, 3U, 3U, 2U};
  std::array<int, kAxes> seen{};

  for (std::size_t batch = 0; batch < kBatches; ++batch) {
    EXPECT_EQ(Access::batchSizes()[batch], expected_sizes[batch]) << "batch " << batch;
    for (std::size_t item = 0; item < Access::batchSizes()[batch]; ++item) {
      const std::int8_t axis = Access::batches()[batch][item];
      ASSERT_GE(axis, 0);
      ASSERT_LT(static_cast<std::size_t>(axis), kAxes);
      ++seen[static_cast<std::size_t>(axis)];
    }
  }
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    EXPECT_EQ(seen[axis], 1) << "axis " << axis << " (" << Access::jointNames()[axis] << ")";
  }
}

TEST_F(EnableManagerFixture, ConfigureRejectsInvalidMotionControllerRegistry)
{
  struct Case
  {
    std::vector<std::string> names;
    std::string default_name;
  };
  const std::vector<Case> cases = {
    {{}, "whole_body_jtc"},
    {{"whole_body_jtc", "whole_body_jtc"}, "whole_body_jtc"},
    {{"whole_body_jtc", ""}, "whole_body_jtc"},
    {{"rolling_trajectory_controller"}, "whole_body_jtc"}};

  for (std::size_t index = 0U; index < cases.size(); ++index) {
    auto invalid = std::make_unique<EnableManagerController>();
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {
          rclcpp::Parameter("motion_controller_names", cases[index].names),
          rclcpp::Parameter("default_motion_controller", cases[index].default_name),
          rclcpp::Parameter("update_rate", 1000),
        });
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    ASSERT_EQ(
      invalid->init("enable_manager_invalid_registry_" + std::to_string(index), "", options),
      controller_interface::return_type::OK);
    EXPECT_EQ(
      invalid->on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::ERROR)
      << "case " << index;
  }
}

TEST_F(EnableManagerFixture, RegistrySwitchPlanDeactivatesEveryOtherCommandWriter)
{
  initAndConfigure();
  controller_manager_msgs::srv::ListControllers::Response states;
  states.controller.resize(2U);
  states.controller[0].name = "whole_body_jtc";
  states.controller[0].state = "inactive";
  states.controller[1].name = "rolling_trajectory_controller";
  states.controller[1].state = "active";

  controller_manager_msgs::srv::SwitchController::Request activate_default;
  EXPECT_EQ(
    Access::buildSwitchRequest(*controller_, states, true, activate_default),
    SwitchResult::kSuccess);
  EXPECT_THAT(activate_default.activate_controllers, ::testing::ElementsAre("whole_body_jtc"));
  EXPECT_THAT(
    activate_default.deactivate_controllers,
    ::testing::ElementsAre("rolling_trajectory_controller"));

  states.controller[0].state = "active";
  controller_manager_msgs::srv::SwitchController::Request deactivate_all;
  EXPECT_EQ(
    Access::buildSwitchRequest(*controller_, states, false, deactivate_all),
    SwitchResult::kSuccess);
  EXPECT_TRUE(deactivate_all.activate_controllers.empty());
  EXPECT_THAT(
    deactivate_all.deactivate_controllers,
    ::testing::ElementsAre("whole_body_jtc", "rolling_trajectory_controller"));

  states.controller.pop_back();
  controller_manager_msgs::srv::SwitchController::Request incomplete;
  EXPECT_EQ(
    Access::buildSwitchRequest(*controller_, states, false, incomplete),
    SwitchResult::kFailed);
}

TEST_F(EnableManagerFixture, RegistryVerificationRequiresExactlyTheRequestedWriters)
{
  initAndConfigure();
  controller_manager_msgs::srv::ListControllers::Response states;
  states.controller.resize(2U);
  states.controller[0].name = "whole_body_jtc";
  states.controller[1].name = "rolling_trajectory_controller";

  states.controller[0].state = "active";
  states.controller[1].state = "inactive";
  EXPECT_TRUE(Access::verifyRegistryState(*controller_, states, true));
  EXPECT_FALSE(Access::verifyRegistryState(*controller_, states, false));

  states.controller[0].state = "inactive";
  EXPECT_TRUE(Access::verifyRegistryState(*controller_, states, false));
  EXPECT_FALSE(Access::verifyRegistryState(*controller_, states, true));

  states.controller[1].state = "active";
  EXPECT_FALSE(Access::verifyRegistryState(*controller_, states, true));
  EXPECT_FALSE(Access::verifyRegistryState(*controller_, states, false));

  states.controller.pop_back();
  EXPECT_FALSE(Access::verifyRegistryState(*controller_, states, false));
}

TEST_F(EnableManagerFixture, DetectsExclusiveMotionControllerOwnership)
{
  initAndConfigure();
  controller_manager_msgs::srv::ListControllers::Response states;
  states.controller.resize(2U);
  states.controller[0].name = "whole_body_jtc";
  states.controller[1].name = "rolling_trajectory_controller";

  states.controller[0].state = "active";
  states.controller[1].state = "inactive";
  EXPECT_EQ(Access::detectControlMode(*controller_, states), ControlMode::kFjtReady);

  states.controller[0].state = "inactive";
  states.controller[1].state = "active";
  EXPECT_EQ(Access::detectControlMode(*controller_, states), ControlMode::kRollingReady);

  states.controller[1].state = "inactive";
  EXPECT_EQ(Access::detectControlMode(*controller_, states), ControlMode::kDisabled);

  states.controller[0].state = "active";
  states.controller[1].state = "active";
  EXPECT_EQ(
    Access::detectControlMode(*controller_, states), ControlMode::kRestartRequired);

  states.controller.pop_back();
  EXPECT_EQ(
    Access::detectControlMode(*controller_, states), ControlMode::kRestartRequired);
}

TEST_F(EnableManagerFixture, ClassifiesNonTransactionalModeSwitchOutcomes)
{
  initAndConfigure();
  controller_manager_msgs::srv::ListControllers::Response states;
  states.controller.resize(2U);
  states.controller[0].name = "whole_body_jtc";
  states.controller[1].name = "rolling_trajectory_controller";

  states.controller[0].state = "inactive";
  states.controller[1].state = "active";
  EXPECT_EQ(
    Access::classifyModeSwitchState(
      *controller_, states, ControlMode::kFjtReady, ControlMode::kRollingReady),
    ModeSwitchState::kTargetActive);

  states.controller[0].state = "active";
  states.controller[1].state = "inactive";
  EXPECT_EQ(
    Access::classifyModeSwitchState(
      *controller_, states, ControlMode::kFjtReady, ControlMode::kRollingReady),
    ModeSwitchState::kSourcePreserved);

  states.controller[0].state = "inactive";
  EXPECT_EQ(
    Access::classifyModeSwitchState(
      *controller_, states, ControlMode::kFjtReady, ControlMode::kRollingReady),
    ModeSwitchState::kAllInactive);

  states.controller[0].state = "active";
  states.controller[1].state = "active";
  EXPECT_EQ(
    Access::classifyModeSwitchState(
      *controller_, states, ControlMode::kFjtReady, ControlMode::kRollingReady),
    ModeSwitchState::kAmbiguous);
}

TEST_F(EnableManagerFixture, StableAndTakeoverGatesUseIndependentPerAxisThresholds)
{
  initAndConfigure();
  std::array<double, kAxes> previous{};
  std::array<double, kAxes> current{};
  current[0] = 0.00002;
  current[13] = 0.000002;
  EXPECT_TRUE(
    Access::stableInterval(*controller_, previous, current, 1000000U, 5000000U));
  EXPECT_FALSE(
    Access::stableInterval(*controller_, previous, current, 1000000U, 10000000U));

  current[0] = 0.001;
  EXPECT_FALSE(
    Access::stableInterval(*controller_, previous, current, 1000000U, 5000000U));

  std::array<double, kAxes> source{};
  std::array<double, kAxes> actual{};
  source[0] = 0.008;
  source[13] = 0.004;
  EXPECT_TRUE(Access::sourceWithinTakeover(*controller_, source, actual));
  source[0] = 0.009;
  EXPECT_FALSE(Access::sourceWithinTakeover(*controller_, source, actual));
}

TEST_F(EnableManagerFixture, ModeServiceIsIdempotentAndFailsBeforeSwitchAdmission)
{
  bringUpToIdle();
  auto observer = std::make_shared<rclcpp::Node>("mode_result_observer");
  std::vector<ModeResult> mode_results;
  auto mode_result_subscription = observer->create_subscription<ModeResult>(
    "/rt/internal/joint_control/mode_result", robot_interfaces_qos::state(),
    [&mode_results](ModeResult::SharedPtr result) {
      mode_results.push_back(*result);
    });
  const auto discovery_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (
    mode_result_subscription->get_publisher_count() == 0U &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(mode_result_subscription->get_publisher_count(), 1U);

  auto request = std::make_shared<ModeService::Request>();
  request->protocol_major = 1U;
  request->protocol_minor = 0U;
  request->client_instance_id.uuid.fill(0x11U);
  request->request_id.uuid.fill(0x22U);
  request->expected_mode.value =
    robot_rt_control_interfaces::msg::JointControlMode::FJT_READY;
  request->target_mode.value =
    robot_rt_control_interfaces::msg::JointControlMode::ROLLING_READY;

  auto first = std::make_shared<ModeService::Response>();
  Access::handleSetMode(*controller_, request, first);
  EXPECT_FALSE(first->accepted);
  EXPECT_EQ(first->result.value, ServiceResult::NOT_ENABLED);
  EXPECT_TRUE(first->error.retryable);
  EXPECT_EQ(first->request_id.uuid, request->request_id.uuid);
  rclcpp::executors::SingleThreadedExecutor observer_executor;
  observer_executor.add_node(observer);
  const auto result_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (mode_results.empty() && std::chrono::steady_clock::now() < result_deadline) {
    observer_executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(mode_results.size(), 1U);
  EXPECT_EQ(mode_results[0].sequence, 1U);
  EXPECT_EQ(mode_results[0].request_id, request->request_id);
  EXPECT_EQ(mode_results[0].result.value, ServiceResult::NOT_ENABLED);
  EXPECT_EQ(
    mode_results[0].mode.value,
    robot_rt_control_interfaces::msg::JointControlMode::DISABLED);

  auto replay = std::make_shared<ModeService::Response>();
  Access::handleSetMode(*controller_, request, replay);
  EXPECT_EQ(replay->result.value, first->result.value);
  EXPECT_EQ(replay->error.code, first->error.code);
  EXPECT_EQ(replay->request_id.uuid, first->request_id.uuid);
  observer_executor.spin_some();
  EXPECT_EQ(mode_results.size(), 1U);

  auto conflicting_request = std::make_shared<ModeService::Request>(*request);
  conflicting_request->protocol_minor = 1U;
  auto conflict = std::make_shared<ModeService::Response>();
  Access::handleSetMode(*controller_, conflicting_request, conflict);
  EXPECT_EQ(conflict->result.value, ServiceResult::WRONG_REQUEST);
  EXPECT_FALSE(conflict->error.retryable);

  auto admission_request = std::make_shared<ModeService::Request>(*request);
  admission_request->request_id.uuid.fill(0x33U);
  Access::setPhase(*controller_, Phase::kEnabled);
  Access::setControlMode(*controller_, ControlMode::kFjtReady);
  Access::setOwner(*controller_, Owner::kNone);
  auto admission = std::make_shared<ModeService::Response>();
  Access::handleSetMode(*controller_, admission_request, admission);
  EXPECT_EQ(admission->result.value, ServiceResult::SOURCE_STATE_STALE);
  EXPECT_EQ(Access::owner(*controller_), Owner::kNone);
}

TEST_F(EnableManagerFixture, ModeServiceExecutesVerifiedStrictSwitch)
{
  bringUpToIdle();
  setAllStatus(kSwOperationEnabled);
  Access::setPhase(*controller_, Phase::kEnabled);
  Access::setControlMode(*controller_, ControlMode::kFjtReady);
  Access::setOwner(*controller_, Owner::kNone);

  auto fake_manager = std::make_shared<rclcpp::Node>("fake_mode_controller_manager");
  std::mutex states_mutex;
  controller_manager_msgs::srv::ListControllers::Response controller_states;
  controller_states.controller.resize(2U);
  controller_states.controller[0].name = "whole_body_jtc";
  controller_states.controller[0].state = "active";
  controller_states.controller[1].name = "rolling_trajectory_controller";
  controller_states.controller[1].state = "inactive";
  std::atomic_uint32_t list_calls{0U};
  std::atomic_uint32_t switch_calls{0U};
  std::atomic_bool strict_request_valid{false};

  auto list_service = fake_manager->create_service<
    controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers",
    [&states_mutex, &controller_states, &list_calls](
      const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request>,
      std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response) {
      ++list_calls;
      std::lock_guard<std::mutex> lock(states_mutex);
      *response = controller_states;
    });
  auto switch_service = fake_manager->create_service<
    controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller",
    [this, &states_mutex, &controller_states, &switch_calls, &strict_request_valid](
      const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
      std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response) {
      ++switch_calls;
      strict_request_valid.store(
        request->strictness ==
        controller_manager_msgs::srv::SwitchController::Request::STRICT &&
        request->activate_controllers ==
        std::vector<std::string>{"rolling_trajectory_controller"} &&
        request->deactivate_controllers == std::vector<std::string>{"whole_body_jtc"});
      {
        std::lock_guard<std::mutex> lock(states_mutex);
        controller_states.controller[0].state = "inactive";
        controller_states.controller[1].state = "active";
      }
      auto rolling_state =
      std::make_shared<robot_rt_control_interfaces::msg::RollingJointControlState>();
      rolling_state->control_mode.value =
      robot_rt_control_interfaces::msg::JointControlMode::ROLLING_READY;
      rolling_state->controller_boot_id.uuid.fill(0xABU);
      rolling_state->desired_positions.fill(0.0);
      rolling_state->has_session = false;
      Access::handleRollingState(*controller_, rolling_state);
      response->ok = true;
    });

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3U);
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(fake_manager);
  ASSERT_TRUE(
    Access::waitForControllerManagerServices(*controller_, std::chrono::seconds(1)));
  std::thread executor_thread([&executor]() {executor.spin();});

  auto jtc_state =
    std::make_shared<control_msgs::msg::JointTrajectoryControllerState>();
  for (const char * joint_name : Access::jointNames()) {
    jtc_state->joint_names.emplace_back(joint_name);
  }
  jtc_state->output.positions.assign(kAxes, 0.0);
  Access::handleJtcState(*controller_, jtc_state);

  std::atomic_bool pump_running{true};
  std::thread update_thread([this, &pump_running]() {
      std::int64_t update_time_ns = 2000000000LL;
      while (pump_running.load()) {
        controller_->update(
          rclcpp::Time(update_time_ns), rclcpp::Duration::from_nanoseconds(4000000LL));
        update_time_ns += 4000000LL;
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
      }
    });

  auto request = std::make_shared<ModeService::Request>();
  request->protocol_major = 1U;
  request->protocol_minor = 0U;
  request->client_instance_id.uuid.fill(0x41U);
  request->request_id.uuid.fill(0x42U);
  request->expected_mode.value =
    robot_rt_control_interfaces::msg::JointControlMode::FJT_READY;
  request->target_mode.value =
    robot_rt_control_interfaces::msg::JointControlMode::ROLLING_READY;
  auto response = std::make_shared<ModeService::Response>();
  Access::handleSetMode(*controller_, request, response);
  auto replay = std::make_shared<ModeService::Response>();
  Access::handleSetMode(*controller_, request, replay);

  pump_running.store(false);
  update_thread.join();
  executor.cancel();
  executor_thread.join();
  executor.remove_node(fake_manager);
  executor.remove_node(controller_->get_node()->get_node_base_interface());

  EXPECT_TRUE(response->accepted);
  EXPECT_EQ(response->result.value, ServiceResult::NONE);
  EXPECT_TRUE(response->source_controller_deactivated);
  EXPECT_TRUE(response->target_controller_activated);
  EXPECT_FALSE(response->restart_required);
  EXPECT_EQ(
    response->mode.value,
    robot_rt_control_interfaces::msg::JointControlMode::ROLLING_READY);
  EXPECT_THAT(response->controller_boot_id.uuid, ::testing::Each(0xABU));
  EXPECT_EQ(Access::controlMode(*controller_), ControlMode::kRollingReady);
  EXPECT_TRUE(strict_request_valid.load());
  EXPECT_EQ(switch_calls.load(), 1U);
  EXPECT_GE(list_calls.load(), 2U);
  EXPECT_EQ(replay->result.value, response->result.value);
  EXPECT_EQ(switch_calls.load(), 1U);
  EXPECT_FALSE(Access::switchInProgress(*controller_));
}

TEST_F(EnableManagerFixture, GroupFaultPreemptsModeAdmissionWithoutClobberingSafetyOwner)
{
  bringUpToIdle();
  setAllStatus(kSwOperationEnabled);
  Access::setPhase(*controller_, Phase::kEnabled);
  Access::setControlMode(*controller_, ControlMode::kFjtReady);
  Access::setOwner(*controller_, Owner::kNone);

  auto jtc_state =
    std::make_shared<control_msgs::msg::JointTrajectoryControllerState>();
  for (const char * joint_name : Access::jointNames()) {
    jtc_state->joint_names.emplace_back(joint_name);
  }
  jtc_state->output.positions.assign(kAxes, 0.0);
  Access::handleJtcState(*controller_, jtc_state);
  controller_->update(
    rclcpp::Time(2000000000LL), rclcpp::Duration::from_nanoseconds(4000000LL));

  auto request = std::make_shared<ModeService::Request>();
  request->protocol_major = 1U;
  request->protocol_minor = 0U;
  request->client_instance_id.uuid.fill(0x51U);
  request->request_id.uuid.fill(0x52U);
  request->expected_mode.value =
    robot_rt_control_interfaces::msg::JointControlMode::FJT_READY;
  request->target_mode.value =
    robot_rt_control_interfaces::msg::JointControlMode::ROLLING_READY;
  auto response = std::make_shared<ModeService::Response>();
  std::thread mode_thread(
    [this, &request, &response]() {
      Access::handleSetMode(*controller_, request, response);
    });

  const auto owner_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (
    Access::owner(*controller_) != Owner::kMode &&
    std::chrono::steady_clock::now() < owner_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (Access::owner(*controller_) != Owner::kMode) {
    mode_thread.join();
    FAIL() << "mode service never acquired ownership";
    return;
  }
  Access::startEmergency(
    *controller_, Stage::kFaultDetected, 0, kSwFault, 2004000000LL);
  mode_thread.join();

  EXPECT_EQ(response->result.value, ServiceResult::NOT_READY);
  EXPECT_TRUE(Access::modeAbortRequested(*controller_));
  EXPECT_EQ(Access::phase(*controller_), Phase::kEmergencyQuickStop);
  EXPECT_EQ(Access::owner(*controller_), Owner::kInternal);
}

TEST_F(EnableManagerFixture, EmergencyControllerStopWaitsForModeSwitchMutex)
{
  initAndConfigure();
  Access::requestEmergencyControllerStop(*controller_);
  Access::setSwitchInProgress(*controller_, true);

  Access::handleNonRtFaultStop(*controller_);

  EXPECT_TRUE(Access::emergencyControllerStopRequested(*controller_));
  EXPECT_FALSE(Access::restartRequired(*controller_));
}

TEST_F(EnableManagerFixture, ActivateRejectsWrongInterfaceCount)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("update_rate", 1000)});
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  ASSERT_EQ(
    controller_->init("enable_manager_short", "", options),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // Assign only 2 of the 14 required interface pairs.
  command_handles_.emplace_back(Access::jointNames()[0], "control_word", &command_buffer_[0]);
  state_handles_.emplace_back(Access::jointNames()[0], "status_word", &status_buffer_[0]);
  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  loaned_commands.emplace_back(command_handles_[0]);
  loaned_states.emplace_back(state_handles_[0]);
  controller_->assign_interfaces(std::move(loaned_commands), std::move(loaned_states));

  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

// ---------------------------------------------------------------------------
// (a) Enable happy path: 5 batches {3,3,3,3,2}, batch-by-batch control words,
//     Operation Enabled on all 14 axes.
// ---------------------------------------------------------------------------

TEST_F(EnableManagerFixture, EnableHappyPathWalksFiveBatchesInOrder)
{
  bringUpToIdle();

  // Enter kEnabling through the same flag the service callback sets.
  Access::setEnableRequest(*controller_, true);
  updateTimes(1);
  ASSERT_EQ(Access::phase(*controller_), Phase::kEnabling);
  ASSERT_EQ(Access::currentBatch(*controller_), 0U);

  // Expected CiA402 command for each state a batch axis passes through.
  struct Step
  {
    std::uint16_t status_word;
    std::uint16_t expected_command;
  };
  const std::array<Step, 3> kProgression{{
    {kSwSwitchOnDisabled, kCwShutdown},
    {kSwReadyToSwitchOn, kCwSwitchOn},
    {kSwSwitchedOn, kCwEnableOperation},
  }};

  for (std::size_t batch = 0; batch < kBatches; ++batch) {
    SCOPED_TRACE("batch " + std::to_string(batch));
    ASSERT_EQ(Access::currentBatch(*controller_), batch);

    // Axes belonging to this batch, and every axis of a not-yet-started batch.
    std::array<bool, kAxes> in_this_batch{};
    std::array<bool, kAxes> in_later_batch{};
    for (std::size_t b = 0; b < kBatches; ++b) {
      for (std::size_t item = 0; item < Access::batchSizes()[b]; ++item) {
        const auto axis = static_cast<std::size_t>(Access::batches()[b][item]);
        if (b == batch) {
          in_this_batch[axis] = true;
        } else if (b > batch) {
          in_later_batch[axis] = true;
        }
      }
    }

    // Walk this batch through SwitchOnDisabled -> ReadyToSwitchOn -> SwitchedOn.
    for (const Step & step : kProgression) {
      for (std::size_t axis = 0; axis < kAxes; ++axis) {
        if (in_this_batch[axis]) {
          setStatus(axis, step.status_word);
        }
      }
      ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
      ASSERT_EQ(Access::phase(*controller_), Phase::kEnabling);

      for (std::size_t axis = 0; axis < kAxes; ++axis) {
        if (in_this_batch[axis]) {
          EXPECT_EQ(command(axis), step.expected_command)
            << "axis " << axis << " (" << Access::jointNames()[axis] << ")";
        } else if (in_later_batch[axis]) {
          // Batch isolation: axes of later batches must stay un-commanded.
          EXPECT_EQ(command(axis), kCwZero)
            << "later-batch axis " << axis << " was commanded during batch " << batch;
        } else {
          // Already-enabled axes are held at Enable Operation.
          EXPECT_EQ(command(axis), kCwEnableOperation) << "previous-batch axis " << axis;
        }
      }
    }

    // Batch reaches Operation Enabled -> controller advances.
    for (std::size_t axis = 0; axis < kAxes; ++axis) {
      if (in_this_batch[axis]) {
        setStatus(axis, kSwOperationEnabled);
      }
    }
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);

    if (batch + 1U == kBatches) {
      // Last batch: hardware side is done, controller waits on the JTC switch.
      EXPECT_EQ(Access::phase(*controller_), Phase::kJtcActivating);
      EXPECT_TRUE(Access::enableHardwareReady(*controller_));
    } else {
      ASSERT_EQ(Access::phase(*controller_), Phase::kInterBatchDelay);
      // inter_batch_delay must elapse before the next batch starts.
      ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
      EXPECT_EQ(Access::currentBatch(*controller_), batch)
        << "advanced before inter_batch_delay elapsed";
      advance(kInterBatchDelay);
      ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
      EXPECT_EQ(Access::phase(*controller_), Phase::kEnabling);
      EXPECT_EQ(Access::currentBatch(*controller_), batch + 1U);
    }
  }

  // All 14 axes report Operation Enabled and are commanded 0x000F.
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    EXPECT_EQ(Access::decode(status(axis)), DriveState::kOperationEnabled) << "axis " << axis;
    EXPECT_EQ(command(axis), kCwEnableOperation) << "axis " << axis;
  }
}

// ---------------------------------------------------------------------------
// (b) Batch timeout: one axis never reaches Operation Enabled. The failure
//     response must carry failed_batch / failed_joint / status_word / stage.
// ---------------------------------------------------------------------------

class EnableBatchTimeoutTest
  : public EnableManagerFixture,
  public ::testing::WithParamInterface<std::size_t>
{
};

TEST_P(EnableBatchTimeoutTest, StallingOneAxisReportsBatchTimeoutWithFailureDetail)
{
  const std::size_t stalled_batch = GetParam();
  // Stall the last axis of the batch under test.
  const auto stalled_axis =
    static_cast<std::size_t>(
    Access::batches()[stalled_batch][Access::batchSizes()[stalled_batch] - 1U]);
  SCOPED_TRACE(
    "batch " + std::to_string(stalled_batch) + " stalled axis " +
    std::to_string(stalled_axis) + " (" + Access::jointNames()[stalled_axis] + ")");

  bringUpToIdle();

  // The stalled axis is frozen at ReadyToSwitchOn: it accepts the shutdown
  // command but never advances, so its batch can never complete.
  std::array<bool, kAxes> stalled{};
  stalled[stalled_axis] = true;
  setStatus(stalled_axis, kSwReadyToSwitchOn);

  // Go through the real service callback so owner_ is taken via the production
  // CAS. Driving enable_request_ directly would leave owner_ == kNone, which
  // changes which result slot finishDownward() publishes to.
  auto pending = callAsync(ServiceKind::kEnable);

  // Run the RT loop until the batch under test is current.
  bool reached_batch = false;
  for (std::size_t i = 0; i < 3000 && !pending->done.load(); ++i) {
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
    stepDriveResponses(stalled);
    if (Access::phase(*controller_) == Phase::kEnabling &&
      Access::currentBatch(*controller_) == stalled_batch)
    {
      reached_batch = true;
      break;
    }
    if (Access::phase(*controller_) == Phase::kInterBatchDelay) {
      advance(kInterBatchDelay);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  ASSERT_TRUE(reached_batch) << "never reached batch " << stalled_batch;
  ASSERT_EQ(Access::owner(*controller_), Owner::kEnable);

  // Let the healthy axes of this batch climb to Operation Enabled so the
  // stalled axis is the only one left behind. updateEnable() reports the FIRST
  // non-Operation-Enabled axis of the batch, so without this the assertion
  // would land on a sibling axis instead of the one under test.
  for (std::size_t i = 0; i < 12U; ++i) {
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
    stepDriveResponses(stalled);
  }
  for (std::size_t item = 0; item < Access::batchSizes()[stalled_batch]; ++item) {
    const auto axis = static_cast<std::size_t>(Access::batches()[stalled_batch][item]);
    if (axis == stalled_axis) {
      continue;
    }
    ASSERT_EQ(Access::decode(status(axis)), DriveState::kOperationEnabled)
      << "sibling axis " << axis << " never enabled";
  }

  // No premature failure: the batch is still open and nothing was published.
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  ASSERT_EQ(Access::phase(*controller_), Phase::kEnabling);
  ASSERT_EQ(Access::currentBatch(*controller_), stalled_batch);
  ASSERT_FALSE(Access::slotReady(Access::enableResult(*controller_)));

  // batch_timeout expires -> staged rollback.
  advance(kBatchTimeout);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  EXPECT_EQ(Access::phase(*controller_), Phase::kRollback);

  // Drive the rollback (and any emergency escalation) to completion. The
  // stalled axis stays frozen, which is what a genuinely stuck drive does.
  ASSERT_TRUE(pumpUntilDone(pending, 4000, stalled, kDisableStageTimeout / 4.0))
    << "enable service never returned after the batch timeout";
  join(pending);

  // The service response must carry the full failure detail.
  EXPECT_FALSE(pending->response->ok);
  EXPECT_EQ(pending->response->stage, std::string("enable_batch_timeout"));
  EXPECT_EQ(pending->response->failed_batch, static_cast<std::int8_t>(stalled_batch));
  EXPECT_EQ(
    pending->response->failed_joint, std::string(Access::jointNames()[stalled_axis]));
  EXPECT_EQ(pending->response->status_word, kSwReadyToSwitchOn);

  // And the underlying slot agrees with the wire response.
  const auto & slot = Access::enableResult(*controller_);
  ASSERT_TRUE(Access::slotReady(slot));
  EXPECT_FALSE(Access::slotOk(slot));
  EXPECT_EQ(Access::slotStage(slot), Stage::kEnableBatchTimeout);
  EXPECT_EQ(Access::slotFailedBatch(slot), static_cast<std::int8_t>(stalled_batch));
  EXPECT_EQ(Access::slotFailedJoint(slot), static_cast<std::int8_t>(stalled_axis));
  EXPECT_EQ(Access::slotStatusWord(slot), kSwReadyToSwitchOn);
}

INSTANTIATE_TEST_SUITE_P(
  AllBatches, EnableBatchTimeoutTest, ::testing::Values(0U, 1U, 2U, 3U, 4U),
  [](const ::testing::TestParamInfo<std::size_t> & info) {
    return "Batch" + std::to_string(info.param);
  });

// ---------------------------------------------------------------------------
// (c) Two-stage fault reset: 0x0080 goes only to axes currently in Fault, and
//     the staged down-sequence starts only once every axis has left Fault.
// ---------------------------------------------------------------------------

class FaultResetTest
  : public EnableManagerFixture,
  public ::testing::WithParamInterface<std::vector<std::size_t>>
{
};

TEST_P(FaultResetTest, ResetDrivesFaultResetBitOnlyToFaultedAxes)
{
  const std::vector<std::size_t> faulted = GetParam();
  std::array<bool, kAxes> is_faulted{};
  for (const std::size_t axis : faulted) {
    ASSERT_LT(axis, kAxes);
    is_faulted[axis] = true;
  }

  bringUpToIdle();
  for (const std::size_t axis : faulted) {
    setStatus(axis, kSwFault);
  }

  // Reset request from kIdle latches the fault set and enters kResetLow.
  Access::setResetRequest(*controller_, true);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  ASSERT_EQ(Access::phase(*controller_), Phase::kResetLow);

  // Stage 1 (kResetLow): every control word driven low, no reset bit yet.
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    EXPECT_EQ(command(axis), kCwZero) << "kResetLow axis " << axis;
  }
  ASSERT_EQ(Access::phase(*controller_), Phase::kResetHigh);

  // Stage 2 (kResetHigh): 0x0080 only to the axes still in Fault.
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  ASSERT_EQ(Access::phase(*controller_), Phase::kResetHigh)
    << "left kResetHigh while axes were still faulted";
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    if (is_faulted[axis]) {
      EXPECT_EQ(command(axis), kCwFaultReset)
        << "faulted axis " << axis << " (" << Access::jointNames()[axis] << ")";
    } else {
      EXPECT_EQ(command(axis), kCwZero)
        << "healthy axis " << axis << " received the fault-reset bit";
    }
  }

  // Clear all but the last faulted axis: the down-sequence must NOT start yet.
  for (std::size_t i = 0; i + 1U < faulted.size(); ++i) {
    setStatus(faulted[i], kSwSwitchOnDisabled);
  }
  if (faulted.size() > 1U) {
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
    EXPECT_EQ(Access::phase(*controller_), Phase::kResetHigh)
      << "started the down-sequence before every axis left Fault";
    const std::size_t last = faulted.back();
    EXPECT_EQ(command(last), kCwFaultReset) << "still-faulted axis lost its reset bit";
    for (std::size_t i = 0; i + 1U < faulted.size(); ++i) {
      EXPECT_EQ(command(faulted[i]), kCwZero) << "recovered axis kept the reset bit";
    }
  }

  // Last axis leaves Fault -> staged down-sequence begins.
  setStatus(faulted.back(), kSwSwitchOnDisabled);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  EXPECT_EQ(Access::phase(*controller_), Phase::kResetDisabling);

  // Down-sequence completes and reports success.
  bool published = false;
  for (std::size_t i = 0; i < 3000; ++i) {
    if (Access::slotReady(Access::resetResult(*controller_))) {
      published = true;
      break;
    }
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
    stepDriveResponses();
  }
  ASSERT_TRUE(published) << "reset never published a result";

  const auto & slot = Access::resetResult(*controller_);
  EXPECT_TRUE(Access::slotOk(slot));
  EXPECT_EQ(Access::slotStage(slot), Stage::kSuccess);
  EXPECT_EQ(Access::phase(*controller_), Phase::kIdle);
  EXPECT_EQ(Access::owner(*controller_), Owner::kNone);
}

INSTANTIATE_TEST_SUITE_P(
  FaultSets, FaultResetTest,
  ::testing::Values(
    std::vector<std::size_t>{0U},
    std::vector<std::size_t>{13U},
    std::vector<std::size_t>{2U, 7U},
    std::vector<std::size_t>{0U, 6U, 12U, 13U}),
  [](const ::testing::TestParamInfo<std::vector<std::size_t>> & info) {
    std::string name = "Axes";
    for (const std::size_t axis : info.param) {
      name += "_" + std::to_string(axis);
    }
    return name;
  });

TEST_F(EnableManagerFixture, ResetWithNoFaultedAxisReportsAlreadyClear)
{
  bringUpToIdle();

  Access::setResetRequest(*controller_, true);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);

  const auto & slot = Access::resetResult(*controller_);
  ASSERT_TRUE(Access::slotReady(slot));
  EXPECT_TRUE(Access::slotOk(slot));
  EXPECT_EQ(Access::slotStage(slot), Stage::kAlreadyClear);
  EXPECT_EQ(Access::owner(*controller_), Owner::kNone);

  RtEnable::Response response;
  Access::fillResponse(*controller_, slot, response);
  EXPECT_EQ(response.stage, std::string("already_clear"));
  EXPECT_EQ(response.failed_batch, -1);
  EXPECT_EQ(response.failed_joint, std::string(""));
}

TEST_F(EnableManagerFixture, ResetTimesOutWhenFaultNeverClears)
{
  constexpr std::size_t kStuckAxis = 4U;
  bringUpToIdle();
  setStatus(kStuckAxis, kSwFault);

  Access::setResetRequest(*controller_, true);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  ASSERT_EQ(Access::phase(*controller_), Phase::kResetLow);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  ASSERT_EQ(Access::phase(*controller_), Phase::kResetHigh);

  // The axis holds Fault through the whole fault_reset_timeout window.
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  ASSERT_FALSE(Access::slotReady(Access::resetResult(*controller_)));
  advance(kFaultResetTimeout);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);

  const auto & slot = Access::resetResult(*controller_);
  ASSERT_TRUE(Access::slotReady(slot));
  EXPECT_FALSE(Access::slotOk(slot));
  EXPECT_EQ(Access::slotStage(slot), Stage::kFaultResetTimeout);
  EXPECT_EQ(Access::slotFailedJoint(slot), static_cast<std::int8_t>(kStuckAxis));
  EXPECT_EQ(Access::slotStatusWord(slot), kSwFault);
  EXPECT_EQ(Access::phase(*controller_), Phase::kFailed);
  EXPECT_EQ(Access::owner(*controller_), Owner::kNone);

  RtEnable::Response response;
  Access::fillResponse(*controller_, slot, response);
  EXPECT_EQ(response.stage, std::string("fault_reset_timeout"));
  EXPECT_EQ(response.failed_joint, std::string(Access::jointNames()[kStuckAxis]));
}

// ---------------------------------------------------------------------------
// (d) Disable staging: ordered down-transitions and disable_stage_timeout.
// ---------------------------------------------------------------------------

TEST_F(EnableManagerFixture, DisableWalksOperationEnabledDownInOrder)
{
  bringUpToIdle();

  // Put the machine in kEnabled with all axes at Operation Enabled, the state
  // a completed enable leaves behind, and hand ownership to the disable path as
  // handleDisable() would via its owner_ CAS.
  setAllStatus(kSwOperationEnabled);
  Access::setPhase(*controller_, Phase::kEnabled);
  Access::setOwner(*controller_, Owner::kDisable);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    ASSERT_EQ(command(axis), kCwEnableOperation) << "axis " << axis;
  }

  // Disable request moves into the staged down-sequence.
  Access::setDisableRequest(*controller_, true);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  ASSERT_EQ(Access::phase(*controller_), Phase::kDisabling);

  // Stage 1: Operation Enabled -> commanded 0x0007 (disable operation).
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    EXPECT_EQ(command(axis), kCwSwitchOn) << "stage-1 axis " << axis;
  }

  // Stage 2: drives report Switched On -> commanded 0x0006 (shutdown).
  setAllStatus(kSwSwitchedOn);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    EXPECT_EQ(command(axis), kCwShutdown) << "stage-2 axis " << axis;
  }

  // Stage 3: drives report Ready To Switch On -> commanded 0x0000.
  setAllStatus(kSwReadyToSwitchOn);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    EXPECT_EQ(command(axis), kCwZero) << "stage-3 axis " << axis;
  }

  // Terminal: Switch On Disabled on every axis completes the disable.
  setAllStatus(kSwSwitchOnDisabled);
  bool published = false;
  for (std::size_t i = 0; i < 200; ++i) {
    if (Access::slotReady(Access::disableResult(*controller_))) {
      published = true;
      break;
    }
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  }
  ASSERT_TRUE(published) << "disable never published a result";

  const auto & slot = Access::disableResult(*controller_);
  EXPECT_TRUE(Access::slotOk(slot));
  EXPECT_EQ(Access::slotStage(slot), Stage::kSuccess);
  EXPECT_EQ(Access::phase(*controller_), Phase::kIdle);
  EXPECT_EQ(Access::owner(*controller_), Owner::kNone);
}

TEST_F(EnableManagerFixture, DisableStageTimeoutEscalatesToEmergencyQuickStop)
{
  constexpr std::size_t kStuckAxis = 5U;
  bringUpToIdle();

  setAllStatus(kSwOperationEnabled);
  Access::setPhase(*controller_, Phase::kEnabled);
  // owner_ == kDisable is what handleDisable() installs via its CAS. It matters
  // here because the stage-timeout escalation records interrupted_owner_ from
  // owner_, and finishDownward() only publishes to the disable slot when that
  // owner is kDisable. The flags are driven directly rather than through the
  // callback so the (kAmbiguous) JTC deactivation cannot mask kDisableTimeout.
  Access::setOwner(*controller_, Owner::kDisable);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);

  Access::setDisableRequest(*controller_, true);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  ASSERT_EQ(Access::phase(*controller_), Phase::kDisabling);

  // One axis refuses to leave Operation Enabled, so stage 0 can never advance.
  // Every other axis is walked all the way down to Switch On Disabled, leaving
  // the stuck axis as the only non-terminal one. updateDownward() reports the
  // FIRST non-terminal axis, so this is what makes the report name kStuckAxis.
  std::array<bool, kAxes> stalled{};
  stalled[kStuckAxis] = true;
  for (std::size_t i = 0; i < 6U; ++i) {
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
    stepDriveResponses(stalled);
    ASSERT_EQ(Access::phase(*controller_), Phase::kDisabling)
      << "left kDisabling before the stage timeout expired";
  }
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    if (axis == kStuckAxis) {
      ASSERT_EQ(Access::decode(status(axis)), DriveState::kOperationEnabled);
    } else {
      ASSERT_EQ(Access::decode(status(axis)), DriveState::kSwitchOnDisabled)
        << "healthy axis " << axis << " did not reach the terminal state";
    }
  }

  // disable_stage_timeout expires -> emergency quick stop.
  advance(kDisableStageTimeout);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  EXPECT_EQ(Access::phase(*controller_), Phase::kEmergencyQuickStop);

  // Quick stop commands 0x0002 to the axis still in Operation Enabled.
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  EXPECT_EQ(command(kStuckAxis), kCwQuickStop);

  // Let the stuck axis finally follow, and confirm the failure result lands.
  bool published = false;
  for (std::size_t i = 0; i < 200; ++i) {
    if (Access::slotReady(Access::disableResult(*controller_))) {
      published = true;
      break;
    }
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
    stepDriveResponses();
  }
  ASSERT_TRUE(published) << "no disable result after the stage timeout escalation";

  const auto & slot = Access::disableResult(*controller_);
  EXPECT_FALSE(Access::slotOk(slot));
  EXPECT_EQ(Access::slotStage(slot), Stage::kDisableTimeout);
  EXPECT_EQ(Access::slotFailedJoint(slot), static_cast<std::int8_t>(kStuckAxis));
  // The status word captured at the timeout, not the one after recovery.
  EXPECT_EQ(Access::slotStatusWord(slot), kSwOperationEnabled);

  RtEnable::Response response;
  Access::fillResponse(*controller_, slot, response);
  EXPECT_EQ(response.stage, std::string("disable_timeout"));
  EXPECT_EQ(response.failed_joint, std::string(Access::jointNames()[kStuckAxis]));
}

TEST_F(EnableManagerFixture, DisableFromIdleReportsAlreadyDisabled)
{
  bringUpToIdle();

  auto pending = callAsync(ServiceKind::kDisable);
  ASSERT_TRUE(pumpUntilDone(pending, 500));
  join(pending);

  EXPECT_TRUE(pending->response->ok);
  EXPECT_EQ(pending->response->stage, std::string("already_disabled"));
  EXPECT_EQ(pending->response->failed_batch, -1);
  EXPECT_EQ(pending->response->failed_joint, std::string(""));
}

// ---------------------------------------------------------------------------
// (e) Owner exclusivity. Asserted against the ACTUAL semantics in the source:
//     each service has a callback-reentrancy latch (*_callback_active_) that
//     rejects a second concurrent call to the SAME service, and a shared
//     owner_ CAS that rejects a different service while one owns the machine.
//     There is no queue: rejections return immediately with
//     stage == "operation_in_progress".
// ---------------------------------------------------------------------------

TEST_F(EnableManagerFixture, ResetIsRejectedWhileEnableOwnsTheMachine)
{
  bringUpToIdle();

  // Enable takes ownership and parks in kEnabling.
  Access::setEnableRequest(*controller_, true);
  Access::setOwner(*controller_, Owner::kEnable);
  ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
  ASSERT_EQ(Access::phase(*controller_), Phase::kEnabling);
  ASSERT_EQ(Access::owner(*controller_), Owner::kEnable);

  // A reset arriving now is refused: phase is not kIdle/kFailed.
  auto pending = callAsync(ServiceKind::kReset);
  ASSERT_TRUE(pumpUntilDone(pending, 500));
  join(pending);

  EXPECT_FALSE(pending->response->ok);
  EXPECT_EQ(pending->response->stage, std::string("operation_in_progress"));
  // Ownership is untouched by the rejected request.
  EXPECT_EQ(Access::owner(*controller_), Owner::kEnable);
}

TEST_F(EnableManagerFixture, EnableIsRejectedWhileAnotherOwnerHoldsTheMachine)
{
  bringUpToIdle();

  // Someone else owns an operation while the phase sits at kIdle. The enable
  // path must fail the owner_ CAS rather than barge in.
  Access::setOwner(*controller_, Owner::kDisable);

  auto pending = callAsync(ServiceKind::kEnable);
  ASSERT_TRUE(pumpUntilDone(pending, 500));
  join(pending);

  EXPECT_FALSE(pending->response->ok);
  EXPECT_EQ(pending->response->stage, std::string("operation_in_progress"));
  EXPECT_EQ(Access::owner(*controller_), Owner::kDisable) << "enable stole ownership";
}

TEST_F(EnableManagerFixture, SecondConcurrentEnableIsRejectedByTheCallbackLatch)
{
  bringUpToIdle();

  // First enable blocks inside its callback waiting on the RT loop.
  auto first = callAsync(ServiceKind::kEnable);
  bool first_took_ownership = false;
  for (std::size_t i = 0; i < 500; ++i) {
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
    if (Access::owner(*controller_) == Owner::kEnable) {
      first_took_ownership = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  ASSERT_TRUE(first_took_ownership) << "first enable never took ownership";

  // A second enable, arriving while the first is still in flight, is rejected
  // immediately by enable_callback_active_ - no queueing.
  auto second = callAsync(ServiceKind::kEnable);
  ASSERT_TRUE(pumpUntilDone(second, 500));
  join(second);
  EXPECT_FALSE(second->response->ok);
  EXPECT_EQ(second->response->stage, std::string("operation_in_progress"));

  // Drain the first call so the fixture can tear down cleanly.
  std::array<bool, kAxes> none{};
  pumpUntilDone(first, 4000, none);
  join(first);
}

TEST_F(EnableManagerFixture, ServicesAreRejectedWhileControllerIsInactive)
{
  initAndConfigure();  // configured but never activated

  for (const ServiceKind kind :
    {ServiceKind::kEnable, ServiceKind::kDisable, ServiceKind::kReset})
  {
    auto pending = callAsync(kind);
    join(pending);
    EXPECT_FALSE(pending->response->ok);
    EXPECT_EQ(pending->response->stage, std::string("controller_inactive"));
  }
}

// ---------------------------------------------------------------------------
// (f) SwitchResult::kAmbiguous. Reachable in unit scope: with no
//     controller_manager present, switchJtc() fails its
//     wait_for_service(0ms) probe and returns kAmbiguous for both directions.
// ---------------------------------------------------------------------------

TEST_F(EnableManagerFixture, SwitchJtcIsAmbiguousWithoutControllerManager)
{
  bringUpToIdle();

  EXPECT_EQ(Access::switchJtc(*controller_, true), SwitchResult::kAmbiguous);
  EXPECT_EQ(Access::switchJtc(*controller_, false), SwitchResult::kAmbiguous);
}

TEST_F(EnableManagerFixture, AmbiguousActivationLeavesRestartRequiredAndRejectsEnable)
{
  bringUpToIdle();
  ASSERT_FALSE(Access::restartRequired(*controller_));

  // Run a full enable through the real callback. The hardware side succeeds, so
  // the callback reaches switchJtc(true), which is kAmbiguous here. Per the
  // source that sets restart_required_ and requests the emergency deactivate.
  auto pending = callAsync(ServiceKind::kEnable);
  ASSERT_TRUE(pumpUntilDone(pending, 6000));
  join(pending);

  EXPECT_FALSE(pending->response->ok);
  EXPECT_TRUE(Access::restartRequired(*controller_))
    << "kAmbiguous activation must latch restart_required_";

  // Once restart_required_ is latched, later enable/reset calls are refused
  // with restart_required until the controller is restarted.
  auto again = callAsync(ServiceKind::kEnable);
  ASSERT_TRUE(pumpUntilDone(again, 2000));
  join(again);
  EXPECT_FALSE(again->response->ok);
  EXPECT_EQ(again->response->stage, std::string("restart_required"));

  auto reset = callAsync(ServiceKind::kReset);
  ASSERT_TRUE(pumpUntilDone(reset, 2000));
  join(reset);
  EXPECT_FALSE(reset->response->ok);
  EXPECT_EQ(reset->response->stage, std::string("restart_required"));
}

// ---------------------------------------------------------------------------
// CiA402 decode table + stage/phase name tables
// ---------------------------------------------------------------------------

struct DecodeCase
{
  std::uint16_t status_word;
  DriveState expected;
  const char * label;
};

class DecodeStateTest : public ::testing::TestWithParam<DecodeCase>
{
};

TEST_P(DecodeStateTest, MapsStatusWordToDriveState)
{
  const DecodeCase & c = GetParam();
  EXPECT_EQ(Access::decode(c.status_word), c.expected) << c.label;
}

INSTANTIATE_TEST_SUITE_P(
  Cia402, DecodeStateTest,
  ::testing::Values(
    DecodeCase{0x0000U, DriveState::kNotReady, "not ready"},
    DecodeCase{0x0040U, DriveState::kSwitchOnDisabled, "switch on disabled"},
    DecodeCase{0x0021U, DriveState::kReadyToSwitchOn, "ready to switch on"},
    DecodeCase{0x0023U, DriveState::kSwitchedOn, "switched on"},
    DecodeCase{0x0027U, DriveState::kOperationEnabled, "operation enabled"},
    DecodeCase{0x0007U, DriveState::kQuickStopActive, "quick stop active"},
    DecodeCase{0x000FU, DriveState::kFaultReactionActive, "fault reaction active"},
    DecodeCase{0x0008U, DriveState::kFault, "fault"},
    // Extra (ignored) bits must not change the decode.
    DecodeCase{0x1237U, DriveState::kOperationEnabled, "operation enabled + extra bits"},
    DecodeCase{0x9608U, DriveState::kFault, "fault + extra bits"},
    // Fault bit set together with bit 6 is not a Fault encoding.
    DecodeCase{0x9648U, DriveState::kUnknown, "fault bit with bit6 set is unknown"}),
  [](const ::testing::TestParamInfo<DecodeCase> & info) {
    return "Sw" + std::to_string(info.param.status_word);
  });

TEST(StageNameTest, EveryStageHasANonEmptyDistinctName)
{
  const std::array<Stage, 18> all{
    Stage::kSuccess, Stage::kAlreadyEnabled, Stage::kAlreadyDisabled, Stage::kAlreadyClear,
    Stage::kOperationInProgress, Stage::kControllerInactive, Stage::kPreemptedByDisable,
    Stage::kEnableBatchTimeout, Stage::kEnableInvalidState, Stage::kFaultDetected,
    Stage::kUnexpectedDriveState, Stage::kDisableTimeout, Stage::kFaultRequiresReset,
    Stage::kFaultResetTimeout, Stage::kJtcActivateFailed, Stage::kJtcDeactivateFailed,
    Stage::kControllerUpdateTimeout, Stage::kRestartRequired};

  std::vector<std::string> names;
  for (const Stage stage : all) {
    const std::string name = Access::stageName(stage);
    EXPECT_FALSE(name.empty());
    EXPECT_NE(name, std::string("unknown"));
    names.push_back(name);
  }
  std::sort(names.begin(), names.end());
  EXPECT_EQ(std::adjacent_find(names.begin(), names.end()), names.end())
    << "duplicate stage name";
}

}  // namespace
}  // namespace enable_manager

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
