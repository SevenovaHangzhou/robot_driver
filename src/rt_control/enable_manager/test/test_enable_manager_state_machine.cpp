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
#include <string>
#include <thread>
#include <vector>

#include "enable_manager/enable_manager_controller.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
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

  static DriveState decode(std::uint16_t sw) {return EnableManagerController::decodeState(sw);}
  static bool isConfirmedDisableTerminal(
    const Controller & controller, std::size_t axis, DriveState state)
  {
    return controller.isConfirmedDisableTerminal(axis, state);
  }
  static const char * stageName(Stage s) {return EnableManagerController::stageName(s);}
  static const char * phaseName(Phase p) {return EnableManagerController::phaseName(p);}
  static SwitchResult switchJtc(Controller & c, bool a) {return c.switchJtc(a);}

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
};

namespace
{

using Access = EnableManagerTestAccess;
using Phase = Access::Phase;
using Owner = Access::Owner;
using Stage = Access::Stage;
using DriveState = Access::DriveState;
using SwitchResult = Access::SwitchResult;
using RtEnable = rt_control_interfaces::srv::RtEnable;

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
  void initAndConfigure(
    const std::string & name = "enable_manager_test",
    const std::vector<std::string> & ready_to_switch_on_terminal_joints = {
      "right_joint2", "right_joint3", "left_joint2", "left_joint3"},
    controller_interface::CallbackReturn expected =
    controller_interface::CallbackReturn::SUCCESS)
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
        rclcpp::Parameter("jtc_name", std::string("whole_body_jtc")),
        rclcpp::Parameter(
          "ready_to_switch_on_disable_terminal_joints",
          ready_to_switch_on_terminal_joints),
        rclcpp::Parameter("update_rate", 1000),
      });
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    ASSERT_EQ(
      controller_->init(name, "", options), controller_interface::return_type::OK);
    ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), expected);
    if (expected == controller_interface::CallbackReturn::SUCCESS) {
      assignInterfaces();
    }
  }

  void assignInterfaces()
  {
    command_handles_.clear();
    state_handles_.clear();
    command_handles_.reserve(kAxes);
    state_handles_.reserve(kAxes);

    // The controller claims interfaces in kJointNames order, so build in that order.
    for (std::size_t axis = 0; axis < kAxes; ++axis) {
      command_handles_.emplace_back(
        Access::jointNames()[axis], "control_word", &command_buffer_[axis]);
      state_handles_.emplace_back(
        Access::jointNames()[axis], "status_word", &status_buffer_[axis]);
    }

    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    loaned_commands.reserve(kAxes);
    loaned_states.reserve(kAxes);
    for (std::size_t axis = 0; axis < kAxes; ++axis) {
      loaned_commands.emplace_back(command_handles_[axis]);
      loaned_states.emplace_back(state_handles_[axis]);
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
    return controller_->update(rclcpp::Time(now_ns_, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt));
  }

  void enterDisablingFromEnabled()
  {
    bringUpToIdle();
    setAllStatus(kSwOperationEnabled);
    Access::setPhase(*controller_, Phase::kEnabled);
    Access::setOwner(*controller_, Owner::kDisable);
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
    Access::setDisableRequest(*controller_, true);
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
    ASSERT_EQ(Access::phase(*controller_), Phase::kDisabling);
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
    setAllStatus(kSwSwitchedOn);
    ASSERT_EQ(callUpdate(), controller_interface::return_type::OK);
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

  bool pumpUntilDone(const std::shared_ptr<PendingCall> & pending, std::size_t max_iterations = 4000)
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
  std::vector<hardware_interface::CommandInterface> command_handles_;
  std::vector<hardware_interface::StateInterface> state_handles_;
  std::int64_t now_ns_{1000000000};
};

// ---------------------------------------------------------------------------
// Fixture sanity / interface contract
// ---------------------------------------------------------------------------

TEST_F(EnableManagerFixture, ClaimsFourteenControlAndStatusInterfaces)
{
  initAndConfigure();

  const auto command_config = controller_->command_interface_configuration();
  const auto state_config = controller_->state_interface_configuration();

  ASSERT_EQ(command_config.names.size(), kAxes);
  ASSERT_EQ(state_config.names.size(), kAxes);
  EXPECT_EQ(
    command_config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    EXPECT_EQ(
      command_config.names[axis], std::string(Access::jointNames()[axis]) + "/control_word");
    EXPECT_EQ(
      state_config.names[axis], std::string(Access::jointNames()[axis]) + "/status_word");
  }
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

TEST_F(EnableManagerFixture, DisableTerminalPolicyIsFrozenPerJointAtConfigure)
{
  initAndConfigure();
  const std::array<std::string, 4> expected_ready_to_switch_on{
    "right_joint2", "right_joint3", "left_joint2", "left_joint3"};

  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    EXPECT_TRUE(
      Access::isConfirmedDisableTerminal(
        *controller_, axis, DriveState::kSwitchOnDisabled)) << Access::jointNames()[axis];
    const bool expected = std::find(
      expected_ready_to_switch_on.begin(), expected_ready_to_switch_on.end(),
      Access::jointNames()[axis]) != expected_ready_to_switch_on.end();
    EXPECT_EQ(
      Access::isConfirmedDisableTerminal(
        *controller_, axis, DriveState::kReadyToSwitchOn), expected)
      << Access::jointNames()[axis];
  }

  ASSERT_TRUE(
    controller_->get_node()->set_parameter(
      rclcpp::Parameter(
        "ready_to_switch_on_disable_terminal_joints",
        std::vector<std::string>{"right_joint1"})).successful);
  EXPECT_FALSE(
    Access::isConfirmedDisableTerminal(
      *controller_, 0U, DriveState::kReadyToSwitchOn));
  EXPECT_TRUE(
    Access::isConfirmedDisableTerminal(
      *controller_, 1U, DriveState::kReadyToSwitchOn));
}

TEST_F(EnableManagerFixture, DisableTerminalPolicyDoesNotDependOnHistoricalAxisIndexes)
{
  initAndConfigure("enable_manager_custom_policy", {"right_joint1"});

  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    EXPECT_EQ(
      Access::isConfirmedDisableTerminal(
        *controller_, axis, DriveState::kReadyToSwitchOn), axis == 0U)
      << Access::jointNames()[axis];
  }
}

TEST_F(EnableManagerFixture, UnknownDisableTerminalJointFailsConfigure)
{
  initAndConfigure(
    "enable_manager_unknown_policy", {"unknown_joint"},
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(EnableManagerFixture, DuplicateDisableTerminalJointFailsConfigure)
{
  initAndConfigure(
    "enable_manager_duplicate_policy", {"right_joint2", "right_joint2"},
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(EnableManagerFixture, ConfiguredReadyToSwitchOnTerminalsCompleteDisable)
{
  enterDisablingFromEnabled();
  setAllStatus(kSwSwitchOnDisabled);
  for (std::size_t axis = 0; axis < kAxes; ++axis) {
    if (Access::isConfirmedDisableTerminal(
        *controller_, axis, DriveState::kReadyToSwitchOn))
    {
      setStatus(axis, kSwReadyToSwitchOn);
    }
  }

  ASSERT_TRUE(spinUntilPhase(Phase::kIdle, 10));
  EXPECT_TRUE(Access::slotReady(Access::disableResult(*controller_)));
  EXPECT_TRUE(Access::slotOk(Access::disableResult(*controller_)));
}

TEST_F(EnableManagerFixture, UnconfiguredReadyToSwitchOnDoesNotCompleteDisable)
{
  enterDisablingFromEnabled();
  setAllStatus(kSwSwitchOnDisabled);
  setStatus(0U, kSwReadyToSwitchOn);

  updateTimes(10);
  EXPECT_EQ(Access::phase(*controller_), Phase::kDisabling);
  EXPECT_FALSE(Access::slotReady(Access::disableResult(*controller_)));
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
