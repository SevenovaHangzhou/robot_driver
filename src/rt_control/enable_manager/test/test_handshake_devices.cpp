// Handshake devices (e.g. the DaMiao head) under the enable_manager state machine.
// The RT update() is driven directly with a virtual clock; a small plugin model follows
// the enable request and reset generation the controller writes.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "enable_manager/enable_manager_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace enable_manager
{

class EnableManagerTestAccess
{
public:
  using Controller = EnableManagerController;
  using Phase = Controller::Phase;
  using Stage = Controller::Stage;
  using ResultSlot = Controller::ResultSlot;

  static Phase phase(const Controller & c) { return c.phase_.load(); }
  static Stage failureStage(const Controller & c) { return c.last_failure_stage_.load(); }
  static void requestEnable(Controller & c) { c.enable_request_.store(true); }
  static void requestDisable(Controller & c) { c.disable_request_.store(true); }
  static void requestReset(Controller & c) { c.reset_request_.store(true); }
  static void activationSucceeded(Controller & c) { c.jtc_activated_request_.store(true); }
  static bool hardwareReady(const Controller & c) { return c.enable_hardware_ready_.load(); }
  static const ResultSlot & enableResult(const Controller & c) { return c.enable_result_; }
  static const ResultSlot & disableResult(const Controller & c) { return c.disable_result_; }
  static const ResultSlot & resetResult(const Controller & c) { return c.reset_result_; }
};

namespace
{

using Access = EnableManagerTestAccess;
using Phase = Access::Phase;
using Stage = Access::Stage;

constexpr std::uint16_t kSwSwitchOnDisabled = 0x0040U;
constexpr std::uint16_t kSwReadyToSwitchOn = 0x0021U;
constexpr std::uint16_t kSwSwitchedOn = 0x0023U;
constexpr std::uint16_t kSwOperationEnabled = 0x0027U;
constexpr double kPhaseDisabled = 0.0;
constexpr double kPhaseEnabled = 2.0;
constexpr double kPhaseFaultLatched = 6.0;

// Device state as the DaMiao plugin reports it; follows the controller's requests.
struct PluginModel
{
  double enable_request{0.0};
  double reset_generation{0.0};
  double phase{kPhaseDisabled};
  double fault_latched{0.0};
  double handled_generation{0.0};
  double motion_allowed{0.0};
  bool stall_enable{false};

  void latch_fault()
  {
    fault_latched = 1.0;
    phase = kPhaseFaultLatched;
    motion_allowed = 0.0;
  }

  void step()
  {
    if (fault_latched == 1.0) {
      if (reset_generation > handled_generation) {
        handled_generation = reset_generation;
        fault_latched = 0.0;
        phase = kPhaseDisabled;
      }
      return;
    }
    handled_generation = reset_generation;
    const bool enable = enable_request == 1.0 && !stall_enable;
    phase = enable ? kPhaseEnabled : kPhaseDisabled;
    motion_allowed = enable ? 1.0 : 0.0;
  }
};

class HandshakeFixture : public ::testing::Test
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

  void TearDown() override
  {
    if (controller_ != nullptr) {
      controller_->release_interfaces();
    }
  }

  // with_axis: one CiA402 axis enabled before the device; otherwise a device-only manager.
  void bringUp(bool with_axis)
  {
    with_axis_ = with_axis;
    std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("handshake_devices", std::vector<std::string>{"head_control"}),
      rclcpp::Parameter("diagnostic_joints", std::vector<std::string>{"head_yaw"}),
      rclcpp::Parameter("diagnostic_joint_interfaces", std::vector<std::string>{"fault_code"}),
      rclcpp::Parameter("jtc_name", std::string("head_position_controller")),
      rclcpp::Parameter("batch_timeout", 0.2),
      rclcpp::Parameter("disable_stage_timeout", 0.2),
      rclcpp::Parameter("fault_reset_timeout", 0.2),
      rclcpp::Parameter("update_rate", 1000)};
    if (with_axis) {
      parameters.emplace_back("managed_joints", std::vector<std::string>{"arm"});
      parameters.emplace_back("enable_batch_joint_names", std::vector<std::string>{"arm"});
      parameters.emplace_back("enable_batch_sizes", std::vector<std::int64_t>{1});
      parameters.emplace_back(
        "ready_to_switch_on_disable_terminal_joints", std::vector<std::string>{});
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    controller_ = std::make_unique<EnableManagerController>();
    ASSERT_EQ(
      controller_->init("handshake_test", "", options), controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);

    commands_.clear();
    states_.clear();
    if (with_axis) {
      commands_.emplace_back("arm", "control_word", &control_word_);
      states_.emplace_back("arm", "status_word", &status_word_);
    }
    commands_.emplace_back("head_control", "enable_request", &plugin_.enable_request);
    commands_.emplace_back("head_control", "reset_generation", &plugin_.reset_generation);
    states_.emplace_back("head_control", "phase", &plugin_.phase);
    states_.emplace_back("head_control", "fault_latched", &plugin_.fault_latched);
    states_.emplace_back("head_control", "handled_reset_generation", &plugin_.handled_generation);
    states_.emplace_back("head_control", "motion_allowed", &plugin_.motion_allowed);
    states_.emplace_back("head_yaw", "fault_code", &fault_code_);
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    for (auto & handle : commands_) {
      loaned_commands.emplace_back(handle);
    }
    for (auto & handle : states_) {
      loaned_states.emplace_back(handle);
    }
    controller_->assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
    ASSERT_EQ(
      controller_->on_activate(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
  }

  void step(std::size_t cycles = 1)
  {
    for (std::size_t i = 0; i < cycles; ++i) {
      now_ns_ += 1000000;
      ASSERT_EQ(
        controller_->update(rclcpp::Time(now_ns_), rclcpp::Duration::from_seconds(0.001)),
        controller_interface::return_type::OK);
      plugin_.step();
      stepDrive();
    }
  }

  bool stepUntil(Phase target, std::size_t limit = 500)
  {
    for (std::size_t i = 0; i < limit && Access::phase(*controller_) != target; ++i) {
      step();
    }
    return Access::phase(*controller_) == target;
  }

  // A healthy CiA402 drive whose status follows the control word.
  void stepDrive()
  {
    switch (static_cast<std::uint16_t>(control_word_)) {
      case 0x0006U:
        status_word_ = kSwReadyToSwitchOn;
        break;
      case 0x0007U:
        status_word_ = kSwSwitchedOn;
        break;
      case 0x000FU:
        status_word_ = kSwOperationEnabled;
        break;
      default:
        status_word_ = kSwSwitchOnDisabled;
        break;
    }
  }

  void enableToEnabled()
  {
    Access::requestEnable(*controller_);
    ASSERT_TRUE(stepUntil(Phase::kJtcActivating));
    ASSERT_TRUE(Access::hardwareReady(*controller_));
    Access::activationSucceeded(*controller_);
    ASSERT_TRUE(stepUntil(Phase::kEnabled));
  }

  std::unique_ptr<EnableManagerController> controller_;
  PluginModel plugin_;
  double control_word_{0.0};
  double status_word_{kSwSwitchOnDisabled};
  double fault_code_{0.0};
  bool with_axis_{false};
  std::vector<hardware_interface::CommandInterface> commands_;
  std::vector<hardware_interface::StateInterface> states_;
  std::int64_t now_ns_{1000000000};
};

TEST_F(HandshakeFixture, DeviceOnlyManagerClaimsOnlyDeviceAndDiagnosticInterfaces)
{
  bringUp(false);
  EXPECT_EQ(
    controller_->command_interface_configuration().names,
    (std::vector<std::string>{"head_control/enable_request", "head_control/reset_generation"}));
  EXPECT_EQ(
    controller_->state_interface_configuration().names,
    (std::vector<std::string>{
      "head_control/phase", "head_control/fault_latched", "head_control/handled_reset_generation",
      "head_control/motion_allowed", "head_yaw/fault_code"}));
}

TEST_F(HandshakeFixture, StartupWithDisabledDeviceReachesIdle)
{
  bringUp(false);
  EXPECT_TRUE(stepUntil(Phase::kIdle));
  EXPECT_EQ(plugin_.enable_request, 0.0);
}

TEST_F(HandshakeFixture, StartupWithLatchedDeviceFaultFails)
{
  bringUp(false);
  plugin_.latch_fault();
  EXPECT_TRUE(stepUntil(Phase::kFailed));
  EXPECT_EQ(Access::failureStage(*controller_), Stage::kFaultRequiresReset);
}

TEST_F(HandshakeFixture, EnableRequestsDeviceAndWaitsForMotionAllowed)
{
  bringUp(false);
  ASSERT_TRUE(stepUntil(Phase::kIdle));
  enableToEnabled();
  EXPECT_EQ(plugin_.enable_request, 1.0);
  EXPECT_TRUE(Access::enableResult(*controller_).ok.load());
}

TEST_F(HandshakeFixture, DeviceThatNeverEnablesRollsBackWithTimeout)
{
  bringUp(false);
  ASSERT_TRUE(stepUntil(Phase::kIdle));
  plugin_.stall_enable = true;
  Access::requestEnable(*controller_);
  step();
  ASSERT_EQ(Access::phase(*controller_), Phase::kEnabling);
  ASSERT_TRUE(stepUntil(Phase::kIdle, 1000));
  EXPECT_FALSE(Access::enableResult(*controller_).ok.load());
  EXPECT_EQ(Access::enableResult(*controller_).stage.load(), Stage::kEnableBatchTimeout);
  EXPECT_EQ(plugin_.enable_request, 0.0);
}

TEST_F(HandshakeFixture, DisableWithdrawsRequestAndReturnsToIdle)
{
  bringUp(false);
  ASSERT_TRUE(stepUntil(Phase::kIdle));
  enableToEnabled();
  Access::requestDisable(*controller_);
  ASSERT_TRUE(stepUntil(Phase::kIdle));
  EXPECT_TRUE(Access::disableResult(*controller_).ok.load());
  EXPECT_EQ(plugin_.enable_request, 0.0);
}

TEST_F(HandshakeFixture, RuntimeDeviceFaultStopsAndFails)
{
  bringUp(false);
  ASSERT_TRUE(stepUntil(Phase::kIdle));
  enableToEnabled();
  plugin_.latch_fault();
  ASSERT_TRUE(stepUntil(Phase::kFailed));
  EXPECT_EQ(plugin_.enable_request, 0.0);
  EXPECT_EQ(Access::failureStage(*controller_), Stage::kFaultDetected);
}

TEST_F(HandshakeFixture, InvalidDeviceStateWhileEnabledIsTreatedAsFault)
{
  bringUp(false);
  ASSERT_TRUE(stepUntil(Phase::kIdle));
  enableToEnabled();
  plugin_.phase = 2.5;
  step();
  EXPECT_NE(Access::phase(*controller_), Phase::kEnabled);
  EXPECT_EQ(plugin_.enable_request, 0.0);
}

TEST_F(HandshakeFixture, ResetAdvancesGenerationAndReturnsToIdle)
{
  bringUp(false);
  plugin_.latch_fault();
  ASSERT_TRUE(stepUntil(Phase::kFailed));
  const double generation = plugin_.reset_generation;
  Access::requestReset(*controller_);
  ASSERT_TRUE(stepUntil(Phase::kIdle));
  EXPECT_EQ(plugin_.reset_generation, generation + 1.0);
  EXPECT_TRUE(Access::resetResult(*controller_).ok.load());
}

TEST_F(HandshakeFixture, MixedTopologyEnablesDeviceOnlyAfterTheLastCia402Batch)
{
  bringUp(true);
  ASSERT_TRUE(stepUntil(Phase::kIdle));
  Access::requestEnable(*controller_);
  while (static_cast<std::uint16_t>(status_word_) != kSwOperationEnabled) {
    EXPECT_EQ(plugin_.enable_request, 0.0) << "device requested before the axis was enabled";
    step();
    ASSERT_LT(now_ns_, 2000000000);
  }
  ASSERT_TRUE(stepUntil(Phase::kJtcActivating));
  EXPECT_EQ(plugin_.enable_request, 1.0);
  Access::activationSucceeded(*controller_);
  ASSERT_TRUE(stepUntil(Phase::kEnabled));
  Access::requestDisable(*controller_);
  ASSERT_TRUE(stepUntil(Phase::kIdle));
  EXPECT_EQ(plugin_.enable_request, 0.0);
  EXPECT_EQ(static_cast<std::uint16_t>(status_word_), kSwSwitchOnDisabled);
}

}  // namespace
}  // namespace enable_manager
