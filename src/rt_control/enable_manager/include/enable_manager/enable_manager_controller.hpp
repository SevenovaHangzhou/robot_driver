#ifndef ENABLE_MANAGER__ENABLE_MANAGER_CONTROLLER_HPP_
#define ENABLE_MANAGER__ENABLE_MANAGER_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "robot_rt_control_interfaces/msg/rolling_joint_control_state.hpp"
#include "robot_rt_control_interfaces/srv/set_joint_control_mode.hpp"
#include "robot_system_interfaces/msg/error_info.hpp"
#include "rt_control_interfaces/msg/joint_control_mode_result.hpp"
#include "rt_control_interfaces/srv/rt_enable.hpp"

namespace enable_manager
{

class EnableManagerController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Test-only access hook: grants the unit test fixture visibility into the private
  // state machine. No behavior change; not referenced by production code.
  friend class EnableManagerTestAccess;

  static constexpr std::size_t kAxisCount = 14U;
  static constexpr std::size_t kBatchCount = 5U;
  static constexpr std::size_t kModeCacheCapacity = 8U;

  enum class Phase : std::uint8_t
  {
    kInactive,
    kStartupSanitizing,
    kIdle,
    kEnabling,
    kInterBatchDelay,
    kJtcActivating,
    kEnabled,
    kJtcDeactivating,
    kDisabling,
    kRollback,
    kResetLow,
    kResetHigh,
    kResetDisabling,
    kEmergencyQuickStop,
    kEmergencyDisable,
    kFailed
  };

  enum class Owner : std::uint8_t {kNone, kEnable, kDisable, kReset, kMode, kInternal};
  enum class SwitchResult : std::uint8_t {kSuccess, kFailed, kAmbiguous};
  enum class ControlMode : std::uint8_t
  {
    kDisabled = 0U,
    kFjtReady = 1U,
    kRollingReady = 2U,
    kRestartRequired = 3U
  };
  enum class ModeSwitchState : std::uint8_t
  {
    kTargetActive,
    kSourcePreserved,
    kAllInactive,
    kAmbiguous
  };
  enum class DriveState : std::uint8_t
  {
    kNotReady,
    kSwitchOnDisabled,
    kReadyToSwitchOn,
    kSwitchedOn,
    kOperationEnabled,
    kQuickStopActive,
    kFaultReactionActive,
    kFault,
    kUnknown
  };

  enum class Stage : std::uint8_t
  {
    kSuccess,
    kAlreadyEnabled,
    kAlreadyDisabled,
    kAlreadyClear,
    kOperationInProgress,
    kControllerInactive,
    kPreemptedByDisable,
    kEnableBatchTimeout,
    kEnableInvalidState,
    kFaultDetected,
    kUnexpectedDriveState,
    kDisableTimeout,
    kFaultRequiresReset,
    kFaultResetTimeout,
    kJtcActivateFailed,
    kJtcDeactivateFailed,
    kControllerUpdateTimeout,
    kRestartRequired
  };

  struct ResultSlot
  {
    std::atomic_bool ready{false};
    std::atomic_bool ok{false};
    std::atomic<std::int8_t> failed_batch{-1};
    std::atomic<std::int8_t> failed_joint{-1};
    std::atomic<std::uint16_t> status_word{0U};
    std::atomic<Stage> stage{Stage::kOperationInProgress};
  };

  struct ActualSample
  {
    std::array<double, kAxisCount> positions{};
    double feedback_age_ms{0.0};
    std::uint64_t steady_time_ns{0U};
    std::uint64_t sequence{0U};
  };

  struct CommandSnapshot
  {
    bool valid{false};
    std::array<double, kAxisCount> positions{};
    std::array<std::uint8_t, 16U> controller_boot_id{};
    std::uint64_t received_steady_ns{0U};
    std::uint8_t control_mode{0U};
    bool has_session{false};
    std::uint8_t session_state{0U};
  };

  using ModeService = robot_rt_control_interfaces::srv::SetJointControlMode;
  using ModeResultMessage = rt_control_interfaces::msg::JointControlModeResult;

  struct ModeCacheEntry
  {
    bool valid{false};
    ModeService::Request request{};
    ModeService::Response response{};
  };

  static DriveState decodeState(std::uint16_t status_word);
  static const char * stageName(Stage stage);
  static const char * phaseName(Phase phase);
  static bool isFaultState(DriveState state);
  static bool isConfirmedDisableTerminal(std::size_t axis, DriveState state);
  static std::uint64_t steadyNowNs() noexcept;
  static std::uint64_t encodeDouble(double value) noexcept;
  static double decodeDouble(std::uint64_t bits) noexcept;
  static bool isZeroIdentifier(
    const std::array<std::uint8_t, 16U> & identifier) noexcept;

  void handleEnable(
    const std::shared_ptr<rt_control_interfaces::srv::RtEnable::Request> request,
    std::shared_ptr<rt_control_interfaces::srv::RtEnable::Response> response);
  void handleDisable(
    const std::shared_ptr<rt_control_interfaces::srv::RtEnable::Request> request,
    std::shared_ptr<rt_control_interfaces::srv::RtEnable::Response> response);
  void handleResetFault(
    const std::shared_ptr<rt_control_interfaces::srv::RtEnable::Request> request,
    std::shared_ptr<rt_control_interfaces::srv::RtEnable::Response> response);
  void handleSetMode(
    const std::shared_ptr<ModeService::Request> request,
    std::shared_ptr<ModeService::Response> response);
  void handleJtcState(
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr message);
  void handleRollingState(
    const robot_rt_control_interfaces::msg::RollingJointControlState::SharedPtr message);
  bool waitForResult(ResultSlot & slot);
  void fillResponse(
    const ResultSlot & slot, rt_control_interfaces::srv::RtEnable::Response & response) const;
  void fillImmediateResponse(
    rt_control_interfaces::srv::RtEnable::Response & response, bool ok, Stage stage) const;
  SwitchResult buildMotionControllerSwitchRequest(
    const controller_manager_msgs::srv::ListControllers::Response & states,
    bool activate_default,
    controller_manager_msgs::srv::SwitchController::Request & request) const;
  bool registeredMotionControllerStateMatches(
    const controller_manager_msgs::srv::ListControllers::Response & states,
    bool default_active) const;
  ControlMode detectControlMode(
    const controller_manager_msgs::srv::ListControllers::Response & states) const;
  ModeSwitchState classifyModeSwitchState(
    const controller_manager_msgs::srv::ListControllers::Response & states,
    ControlMode source, ControlMode target) const;
  bool stableActualInterval(
    const ActualSample & previous, const ActualSample & current) const noexcept;
  bool sourceWithinTakeoverTolerance(
    const std::array<double, kAxisCount> & source,
    const std::array<double, kAxisCount> & actual) const noexcept;
  static bool sameModeRequest(
    const ModeService::Request & lhs, const ModeService::Request & rhs) noexcept;
  ModeCacheEntry * findModeCache(
    const std::array<std::uint8_t, 16U> & client_id,
    const std::array<std::uint8_t, 16U> & request_id) noexcept;
  ModeCacheEntry & allocateModeCache() noexcept;
  bool readActualSample(ActualSample & sample) const noexcept;
  bool readCommandSnapshot(ControlMode mode, CommandSnapshot & snapshot) const;
  std::uint8_t validateModeSource(ControlMode source, CommandSnapshot & snapshot);
  bool queryControllerStates(
    controller_manager_msgs::srv::ListControllers::Response & states,
    std::chrono::milliseconds timeout);
  std::uint8_t executeModeSwitch(
    ControlMode source, ControlMode target, ModeService::Response & response,
    std::uint64_t switch_started_ns);
  bool waitForRollingActivationEvidence(
    std::uint64_t not_before_ns, CommandSnapshot & snapshot);
  void convergeAfterUnsafeModeSwitch(bool restart_required);
  bool releaseModeOwnership() noexcept;
  void setModeResponse(
    ModeService::Response & response, const ModeService::Request & request,
    std::uint8_t result, ControlMode mode, bool source_deactivated,
    bool target_activated, bool restart_required);
  static void populateModeError(
    robot_system_interfaces::msg::ErrorInfo & error, std::uint8_t result);
  void publishModeResult(const ModeService::Response & response);
  static const char * controllerNameForMode(
    ControlMode mode, const std::string & default_controller,
    const std::string & rolling_controller) noexcept;
  SwitchResult switchJtc(bool activate);
  void handleNonRtFaultStop();
  void publishDiagnostics();

  void publishResult(
    ResultSlot & slot, bool ok, Stage stage, std::int8_t failed_batch = -1,
    std::int8_t failed_joint = -1, std::uint16_t status_word = 0U);
  void recordFailure(
    Stage stage, std::int8_t failed_batch, std::int8_t failed_joint,
    std::uint16_t status_word);
  void clearFailure();
  void startDownward(Phase phase, std::int64_t now_ns);
  void updateDownward(std::int64_t now_ns);
  void startEmergency(
    Stage stage, std::int8_t failed_joint, std::uint16_t status_word,
    std::int64_t now_ns);
  void updateEmergencyQuickStop(std::int64_t now_ns);
  void finishDownward();
  void updateEnable(std::int64_t now_ns);
  void updateReset(std::int64_t now_ns);
  void setAllControlWords(std::uint16_t control_word);
  bool allAxesInState(DriveState state) const;
  std::int8_t firstAxisNotInState(DriveState state) const;
  void refreshStatusWords();

  static const std::array<const char *, kAxisCount> kJointNames;
  static const std::array<std::array<std::int8_t, 3>, kBatchCount> kEnableBatches;
  static const std::array<std::uint8_t, kBatchCount> kBatchSizes;

  std::atomic_bool active_{false};
  std::atomic<Phase> phase_{Phase::kInactive};
  std::atomic<Owner> owner_{Owner::kInternal};
  std::atomic_bool enable_request_{false};
  std::atomic_bool disable_request_{false};
  std::atomic_bool reset_request_{false};
  std::atomic_bool enable_hardware_ready_{false};
  std::atomic_bool jtc_activate_failed_request_{false};
  std::atomic_bool emergency_jtc_deactivate_request_{false};
  std::atomic_bool jtc_deactivation_required_{false};
  std::atomic_bool restart_required_{false};
  std::atomic_bool switch_in_progress_{false};
  std::atomic_bool enable_callback_active_{false};
  std::atomic_bool disable_callback_active_{false};
  std::atomic_bool reset_callback_active_{false};
  std::atomic_bool mode_callback_active_{false};
  std::atomic_bool mode_abort_requested_{false};
  std::atomic<ControlMode> current_control_mode_{ControlMode::kDisabled};
  std::array<std::atomic<std::uint64_t>, kAxisCount> actual_position_bits_{};
  std::atomic<std::uint64_t> actual_feedback_age_bits_{0U};
  std::atomic<std::uint64_t> actual_sample_time_ns_{0U};
  std::atomic<std::uint64_t> actual_sample_sequence_{0U};
  std::atomic<std::uint64_t> actual_sample_version_{0U};

  ResultSlot enable_result_;
  ResultSlot disable_result_;
  ResultSlot reset_result_;
  std::atomic<Stage> last_failure_stage_{Stage::kSuccess};
  std::atomic<std::int8_t> last_failed_batch_{-1};
  std::atomic<std::int8_t> last_failed_joint_{-1};
  std::atomic<std::uint16_t> last_failed_status_word_{0U};

  std::array<std::uint16_t, kAxisCount> status_words_{};
  std::array<bool, kAxisCount> reset_targets_{};
  std::size_t current_batch_{0U};
  std::uint8_t downward_stage_{0U};
  std::int64_t stage_deadline_ns_{0};
  bool enable_preempt_requested_{false};
  Owner interrupted_owner_{Owner::kNone};
  std::atomic_bool jtc_deactivate_failed_{false};
  Stage primary_failure_stage_{Stage::kSuccess};
  std::int8_t primary_failed_batch_{-1};
  std::int8_t primary_failed_joint_{-1};
  std::uint16_t primary_failed_status_word_{0U};

  double batch_timeout_seconds_{4.0};
  double disable_stage_timeout_seconds_{4.0};
  double inter_batch_delay_seconds_{0.2};
  double fault_reset_timeout_seconds_{4.0};
  double controller_switch_timeout_seconds_{4.0};
  std::chrono::milliseconds service_result_timeout_{30000};
  std::vector<std::string> motion_controller_names_{
    "whole_body_jtc", "rolling_trajectory_controller"};
  std::string default_motion_controller_{"whole_body_jtc"};
  std::string rolling_motion_controller_{"rolling_trajectory_controller"};
  std::uint64_t mode_switch_maximum_sample_period_ns_{8000000U};
  std::uint64_t mode_switch_source_state_max_age_ns_{100000000U};
  std::size_t mode_switch_stable_interval_count_{5U};
  std::chrono::milliseconds mode_switch_timeout_{500};
  double mode_switch_feedback_age_limit_ms_{500.0};
  std::array<double, kAxisCount> mode_switch_stable_velocity_thresholds_{
    0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
    0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
    0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
    0.00872664626, 0.001};
  std::array<double, kAxisCount> mode_switch_takeover_tolerances_{
    0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
    0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
    0.00872664626, 0.00872664626, 0.00872664626, 0.00872664626,
    0.00872664626, 0.005};
  mutable std::mutex command_snapshot_mutex_{};
  CommandSnapshot jtc_command_snapshot_{};
  CommandSnapshot rolling_command_snapshot_{};
  std::array<ModeCacheEntry, kModeCacheCapacity> mode_cache_{};
  std::size_t next_mode_cache_slot_{0U};
  std::atomic<std::uint64_t> mode_result_sequence_{0U};

  rclcpp::CallbackGroup::SharedPtr enable_callback_group_;
  rclcpp::CallbackGroup::SharedPtr disable_callback_group_;
  rclcpp::CallbackGroup::SharedPtr reset_callback_group_;
  rclcpp::CallbackGroup::SharedPtr mode_callback_group_;
  rclcpp::CallbackGroup::SharedPtr worker_callback_group_;
  rclcpp::Service<rt_control_interfaces::srv::RtEnable>::SharedPtr enable_service_;
  rclcpp::Service<rt_control_interfaces::srv::RtEnable>::SharedPtr disable_service_;
  rclcpp::Service<rt_control_interfaces::srv::RtEnable>::SharedPtr reset_service_;
  rclcpp::Service<ModeService>::SharedPtr mode_service_;
  rclcpp::Publisher<ModeResultMessage>::SharedPtr mode_result_publisher_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    jtc_state_subscription_;
  rclcpp::Subscription<robot_rt_control_interfaces::msg::RollingJointControlState>::SharedPtr
    rolling_state_subscription_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr worker_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace enable_manager

#endif  // ENABLE_MANAGER__ENABLE_MANAGER_CONTROLLER_HPP_
