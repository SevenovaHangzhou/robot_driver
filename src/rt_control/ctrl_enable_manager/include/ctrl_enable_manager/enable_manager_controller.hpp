#ifndef CTRL_ENABLE_MANAGER__ENABLE_MANAGER_CONTROLLER_HPP_
#define CTRL_ENABLE_MANAGER__ENABLE_MANAGER_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
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
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "robot_rt_control_interfaces/msg/rolling_joint_control_state.hpp"
#include "robot_rt_control_interfaces/srv/set_joint_control_mode.hpp"
#include "robot_system_interfaces/msg/error_info.hpp"
#include "ctrl_enable_manager/handshake_device.hpp"
#include "robot_rt_control_interfaces/srv/rt_enable.hpp"
#include "hw_semantic_components/cia402_axis.hpp"
#include "infra_shm_ring/shm_ring.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ctrl_enable_manager/emergency_stop_input.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ctrl_enable_manager
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

  // Motion axes of the mode switch: motion_joints, at most kMaxMotionAxes. The per-axis
  // parameter defaults cover the dual arm.
  static constexpr std::size_t kMaxMotionAxes = 16U;
  // Motion groups switch modes independently (e.g. the arms and the chassis); one enable state.
  static constexpr std::size_t kMaxMotionGroups = 4U;
  static constexpr std::size_t kDualArmAxisCount = 14U;
  // CiA402 0x6060 values the mode switch writes to the switched drives (mode_switch_joints).
  static constexpr std::int16_t kCspMode = 8;
  using MotionPositions = std::array<double, kMaxMotionAxes>;
  // EtherCAT AL state Operational (ETG.1000.6), as the slave al_state interface reports it.
  static constexpr double kAlStateOp = 8.0;

  enum class Phase : std::uint8_t {
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

  enum class Owner : std::uint8_t { kNone, kEnable, kDisable, kReset, kMode, kInternal };
  enum class SwitchResult : std::uint8_t { kSuccess, kFailed, kAmbiguous };
  enum class ControlMode : std::uint8_t {
    kDisabled = 0U,
    kFjtReady = 1U,
    kRollingReady = 2U,
    kRestartRequired = 3U,
    kVelocityReady = 4U
  };
  enum class ModeSwitchState : std::uint8_t {
    kTargetActive,
    kSourcePreserved,
    kAllInactive,
    kAmbiguous
  };
  using DriveState = hw_semantic_components::Cia402State;

  enum class Stage : std::uint8_t {
    kSuccess,
    kAlreadyEnabled,
    kAlreadyDisabled,
    kAlreadyClear,
    kOperationInProgress,
    kControllerInactive,
    kPreemptedByDisable,
    kEnableBatchTimeout,
    kEnableInvalidState,
    kModeMismatch,
    kModeNotConfirmed,
    kStateSourcesNotReady,
    kMotionFault,
    kFaultDetected,
    kUnexpectedDriveState,
    kDisableTimeout,
    kFaultRequiresReset,
    kFaultResetTimeout,
    kJtcActivateFailed,
    kJtcDeactivateFailed,
    kControllerUpdateTimeout,
    kRestartRequired,
    kDeviceOffline,
    kEmergencyStop
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
    MotionPositions positions{};
    double feedback_age_ms{0.0};
    std::uint64_t steady_time_ns{0U};
    std::uint64_t sequence{0U};
  };

  struct CommandSnapshot
  {
    bool valid{false};
    MotionPositions positions{};
    std::uint64_t received_steady_ns{0U};
    std::uint8_t control_mode{0U};
    // Rolling only: RollingJointControlState.state; the rolling source must be holding.
    std::uint8_t rolling_state{0U};
  };

  using ModeService = robot_rt_control_interfaces::srv::SetJointControlMode;

  // Mode-switch outcome; populateModeError maps it to the public ErrorInfo.
  enum ModeResult : std::uint8_t {
    kModeOk = 0U,
    kModeNotEnabled,
    kModeNotReady,
    kModeWrongMode,
    kModeSourceStale,
    kModeSourceMoving,
    kModeFeedbackStale,
    kModeTakeoverMismatch,
    kModeSwitchRejected,
    kModeRestartRequired,
    kModeDriveModeTimeout
  };

  // One motion group: the controllers its mode service switches between, the joints whose
  // standstill admits a switch, and the drives whose operation mode follows the controller set.
  struct MotionGroup
  {
    // Parameter prefix and the namespace of the group's standstill service.
    std::string name;
    std::string default_controller;
    std::string rolling_controller;
    // Velocity-mode groups (the swerve chassis): default_controller runs VELOCITY_READY and this
    // trajectory controller FJT_READY; empty for the arms, where default_controller is FJT_READY.
    std::string trajectory_controller;
    ControlMode default_mode{ControlMode::kFjtReady};
    bool mode_switching{false};
    std::string service_name;
    std::atomic<ControlMode> current_mode{ControlMode::kDisabled};
    std::atomic_bool callback_active{false};
    std::atomic_bool abort_requested{false};

    // The group's own controllers (default, then the alternate); companions are not listed.
    std::vector<std::string> controllers;
    std::vector<std::string> joint_names;
    // Index of the first position state interface; the shared EtherCAT age follows all groups.
    std::size_t state_offset{0U};
    // Realtime writers and service readers share these; the doubles must stay lock-free.
    std::array<std::atomic<double>, kMaxMotionAxes> sample_positions{};
    std::atomic<double> sample_feedback_age_ms{0.0};
    std::atomic<std::uint64_t> sample_time_ns{0U};
    std::atomic<std::uint64_t> sample_sequence{0U};
    std::atomic<std::uint64_t> sample_version{0U};
    std::uint64_t maximum_sample_period_ns{2000000U};
    std::uint64_t source_state_max_age_ns{100000000U};
    std::size_t stable_interval_count{5U};
    std::chrono::milliseconds switch_timeout{500};
    double feedback_age_limit_ms{500.0};
    MotionPositions stable_velocity_thresholds{};
    MotionPositions takeover_tolerances{};
    mutable std::mutex snapshot_mutex{};
    CommandSnapshot jtc_snapshot{};
    CommandSnapshot rolling_snapshot{};

    // Drives whose operation mode follows the controller set; enable checks their startup mode.
    std::vector<std::string> drive_joint_names;
    std::vector<std::size_t> drive_axes;
    std::vector<std::uint8_t> drive_axis;
    std::size_t drive_command_offset{0U};
    // CSP requested by the mode switch; the realtime update clears it whenever the group is not
    // enabled, so every enable starts in the startup mode.
    std::atomic_bool drive_csp_requested{false};
    std::array<std::atomic<std::int16_t>, kMaxMotionAxes> drive_mode_display{};

    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::Service<ModeService>::SharedPtr mode_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr standstill_service;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
      jtc_state_subscription;
    rclcpp::Subscription<robot_rt_control_interfaces::msg::RollingJointControlState>::SharedPtr
      rolling_state_subscription;
  };
  static_assert(std::atomic<double>::is_always_lock_free);

  static DriveState decodeState(std::uint16_t status_word);
  static const char * stageName(Stage stage);
  static const char * phaseName(Phase phase);
  static bool isFaultState(DriveState state);
  static std::uint64_t steadyNowNs() noexcept;
  bool isConfirmedDisableTerminal(std::size_t axis, DriveState state) const;
  bool configureTopology();
  rcl_interfaces::msg::SetParametersResult validateTopologyParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);

  void handleEnable(
    const std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Request> request,
    std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Response> response);
  void handleDisable(
    const std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Request> request,
    std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Response> response);
  void handleResetFault(
    const std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Request> request,
    std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Response> response);
  void handleSetMode(
    MotionGroup & group, const std::shared_ptr<ModeService::Request> request,
    std::shared_ptr<ModeService::Response> response);
  void handleConfirmStandstill(
    MotionGroup & group, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleJtcState(
    MotionGroup & group,
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr message);
  void handleRollingState(
    MotionGroup & group,
    const robot_rt_control_interfaces::msg::RollingJointControlState::SharedPtr message);
  bool waitForResult(ResultSlot & slot);
  void fillResponse(
    const ResultSlot & slot, robot_rt_control_interfaces::srv::RtEnable::Response & response) const;
  void fillImmediateResponse(
    robot_rt_control_interfaces::srv::RtEnable::Response & response, bool ok, Stage stage) const;
  // Public DREE error for a finished request, logged as "<service> accepted|rejected: ...".
  void reportResult(
    const char * service, robot_rt_control_interfaces::srv::RtEnable::Response & response,
    std::uint32_t failure_code) const;
  SwitchResult buildControllerSwitchRequest(
    const MotionGroup & group,
    const controller_manager_msgs::srv::ListControllers::Response & states, ControlMode target,
    controller_manager_msgs::srv::SwitchController::Request & request) const;
  bool controllerStateMatches(
    const MotionGroup & group,
    const controller_manager_msgs::srv::ListControllers::Response & states,
    ControlMode target) const;
  bool controllerRunsInMode(
    const MotionGroup & group, const std::string & name, ControlMode target) const;
  ControlMode detectControlMode(
    const MotionGroup & group,
    const controller_manager_msgs::srv::ListControllers::Response & states) const;
  ModeSwitchState classifyModeSwitchState(
    const MotionGroup & group,
    const controller_manager_msgs::srv::ListControllers::Response & states, ControlMode source,
    ControlMode target) const;
  // True when the displacement rate from window_start to current is below the thresholds.
  bool stableActualInterval(
    const MotionGroup & group, const ActualSample & window_start,
    const ActualSample & current) const noexcept;
  bool sourceWithinTakeoverTolerance(
    const MotionGroup & group, const MotionPositions & source,
    const MotionPositions & actual) const noexcept;
  bool motionStatesBound(const MotionGroup & group) const;
  void resetMotionGroup(MotionGroup & group);
  void sampleMotionGroup(MotionGroup & group) noexcept;
  bool readActualSample(const MotionGroup & group, ActualSample & sample) const noexcept;
  bool readCommandSnapshot(
    const MotionGroup & group, ControlMode mode, CommandSnapshot & snapshot) const;
  std::uint8_t waitForStandstill(
    MotionGroup & group, Phase phase, Owner owner, ActualSample & sample);
  std::uint8_t validateModeSource(
    MotionGroup & group, ControlMode source, CommandSnapshot & snapshot);
  bool queryControllerStates(
    controller_manager_msgs::srv::ListControllers::Response & states,
    std::chrono::milliseconds timeout);
  std::uint8_t executeModeSwitch(MotionGroup & group, ControlMode source, ControlMode target);
  void convergeAfterUnsafeModeSwitch(MotionGroup & group, bool restart_required);
  bool releaseModeOwnership() noexcept;
  // The mode a controller set implies for the switched drives: CSP while the trajectory
  // controller owns a velocity-mode group, else each drive's startup mode.
  bool switchedDrivesUseCsp(const MotionGroup & group, ControlMode mode) const noexcept;
  // Requests the drive mode for `mode`, then waits until every switched drive reports it.
  bool switchDriveModes(MotionGroup & group, ControlMode mode);
  bool configureDriveModeSwitching(MotionGroup & group);
  bool configureMotionGroups();
  bool configureMotionGroup(MotionGroup & group);
  void createMotionGroupEndpoints(MotionGroup & group);
  // Enable/disable plan over every group (each in its default mode) and the companions.
  bool controllerRunsEnabled(const std::string & name) const;
  SwitchResult buildEnableSwitchRequest(
    const controller_manager_msgs::srv::ListControllers::Response & states, bool activate,
    controller_manager_msgs::srv::SwitchController::Request & request) const;
  bool enableStateMatches(
    const controller_manager_msgs::srv::ListControllers::Response & states, bool activate) const;
  void storeGroupModes(ControlMode mode) noexcept;
  void storeEnabledGroupModes() noexcept;
  void abortModeSwitches() noexcept;
  bool bindDriveModeCommands(MotionGroup & group);
  void writeDriveModes(MotionGroup & group) noexcept;
  bool runningModeConfirmed(std::size_t axis) const noexcept;
  static void populateModeError(
    robot_system_interfaces::msg::ErrorInfo & error, std::uint8_t result);
  SwitchResult switchJtc(bool activate);
  void handleNonRtFaultStop();
  bool configureEmergencyStop();
  bool configureEmergencyDisableVoltage();
  std::uint16_t emergencyStopCommand(std::size_t axis, DriveState state) const noexcept;
  bool emergencyStopBlocksEnable() const noexcept;
  bool emergencyStopBlocksReset() const noexcept;
  void updateEmergencyStop(std::int64_t now_ns);
  EmergencyStopInput emergency_stop_input_;
  bool emergency_stop_response_{false};
  std::atomic_bool emergency_stop_latched_{false};
  rclcpp::CallbackGroup::SharedPtr emergency_stop_group_;
  rclcpp::Subscription<EmergencyStopInput::Message>::SharedPtr emergency_stop_subscription_;
  void publishDiagnostics();
  void configureTrace();
  void writeTrace(std::int64_t now_ns, std::int64_t period_ns);

  // Runtime-LOG trace fields, in the order writeTrace() writes them (enum values as numbers).
  static constexpr std::array<const char *, 22> kTraceFields = {
    "phase",
    "owner",
    "current_batch",
    "downward_stage",
    "control_mode",
    "stage_deadline_remaining_ms",
    "last_failure_stage",
    "last_failed_batch",
    "last_failed_joint",
    "last_failed_status_word",
    "primary_failure_stage",
    "primary_failed_batch",
    "primary_failed_joint",
    "motion_fault_pending",
    "enable_request",
    "disable_request",
    "reset_request",
    "switch_in_progress",
    "restart_required",
    "state_sources_qualified",
    "enable_hardware_ready",
    "jtc_deactivate_failed"};

  void publishResult(
    ResultSlot & slot, bool ok, Stage stage, std::int8_t failed_batch = -1,
    std::int8_t failed_joint = -1, std::uint16_t status_word = 0U);
  void recordFailure(
    Stage stage, std::int8_t failed_batch, std::int8_t failed_joint, std::uint16_t status_word);
  void clearFailure();
  void startDownward(Phase phase, std::int64_t now_ns);
  void updateDownward(std::int64_t now_ns);
  void startEmergency(
    Stage stage, std::int8_t failed_joint, std::uint16_t status_word, std::int64_t now_ns);
  void updateEmergencyQuickStop(std::int64_t now_ns);
  void finishDownward();
  void updateEnable(std::int64_t now_ns);
  void updateReset(std::int64_t now_ns);
  void setControlWord(std::size_t axis, std::uint16_t control_word);
  void setAllControlWords(std::uint16_t control_word);
  void releaseCia402Interfaces() noexcept;
  bool allAxesInState(DriveState state) const;
  std::int8_t firstAxisNotInState(DriveState state) const;
  void refreshStatusWords();
  void refreshModes();
  bool modeConfirmed(std::size_t axis) const noexcept;
  bool modePending(std::size_t axis) const noexcept;
  void failEnable(Stage stage, std::ptrdiff_t axis, std::int64_t now_ns);
  // Handshake devices join after the last CiA402 batch and leave with every downward path.
  void updateDeviceEnable(std::int64_t now_ns);
  void requestDevicesEnable(bool enable) noexcept;
  void startDeviceResets() noexcept;
  bool anyDeviceFaulted() const noexcept;
  bool allDevicesEnabled() const noexcept;
  bool allDevicesDisabledOrFaulted() const noexcept;
  bool allDeviceResetsComplete() const noexcept;
  // Index into offline_source_names_ of the first stale age or non-OP slave, -1 when every source is fresh.
  int offlineSource() const noexcept;
  bool rejectIfOffline(ResultSlot & slot);
  bool stateSourcesReady() const noexcept;
  void handleMotionFault();

  std::atomic_bool active_{false};
  std::atomic<Phase> phase_{Phase::kInactive};
  std::atomic<Owner> owner_{Owner::kInternal};
  std::atomic_bool enable_request_{false};
  std::atomic_bool disable_request_{false};
  std::atomic_bool reset_request_{false};
  std::atomic_bool enable_hardware_ready_{false};
  std::atomic_bool jtc_activate_failed_request_{false};
  std::atomic_bool jtc_activated_request_{false};
  std::atomic_bool emergency_jtc_deactivate_request_{false};
  std::atomic_bool jtc_deactivation_required_{false};
  std::atomic_bool restart_required_{false};
  std::atomic_bool switch_in_progress_{false};
  std::atomic_bool enable_callback_active_{false};
  std::atomic_bool disable_callback_active_{false};
  std::atomic_bool reset_callback_active_{false};
  std::atomic_bool topology_parameters_frozen_{false};
  std::atomic<std::uint8_t> topology_parameters_explicit_mask_{0U};

  ResultSlot enable_result_;
  ResultSlot disable_result_;
  ResultSlot reset_result_;
  std::atomic<Stage> last_failure_stage_{Stage::kSuccess};
  std::atomic<std::int8_t> last_failed_batch_{-1};
  std::atomic<std::int8_t> last_failed_joint_{-1};
  std::atomic<std::uint16_t> last_failed_status_word_{0U};
  // 256 denotes an invalid/unavailable mode sample; 0 remains a real pending mode.
  std::atomic<std::int16_t> last_failed_mode_{256};
  std::atomic<std::int16_t> last_expected_mode_{0};

  std::vector<std::string> joint_names_;
  std::vector<std::vector<std::size_t>> enable_batches_;
  std::vector<std::uint8_t> ready_to_switch_on_disable_terminal_;
  // Axes that an emergency stop disables with Disable Voltage (0x0005) instead of Quick Stop:
  // the Tongyi XMC_ESC keeps the motor energized after 0x0002 and then ignores the disable
  // sequence; 0x0005 disables it at once, de-energizes the motor and engages the brake (vendor).
  std::vector<std::uint8_t> emergency_disable_voltage_;
  std::vector<std::uint16_t> status_words_;
  std::vector<std::int16_t> expected_modes_;
  std::vector<std::int16_t> actual_modes_;
  std::vector<std::size_t> mode_state_indices_;
  std::vector<std::uint8_t> mode_confirmed_;
  // Required state sources (e.g. external encoders) gate the first enable batch.
  std::vector<std::string> ready_state_names_;
  std::vector<std::size_t> ready_state_indices_;
  // Bus/device feedback ages (ms) checked before an enable or reset starts: a powered-off drive
  // keeps its last status word, so its state must not be judged while its feedback is stale.
  // Feedback ages first, then EtherCAT slave AL states: a re-powered slave can stay in SAFEOP
  // with fresh process data (2026-11-04 bench); only a stack restart brings it back to OP.
  std::vector<std::string> offline_source_names_;
  std::vector<std::size_t> offline_source_indices_;
  std::size_t age_source_count_{0U};
  // Current offline source, refreshed every cycle; offline_feedback_ keeps the one that stopped
  // or rejected the last request.
  std::atomic<int> offline_now_{-1};
  double feedback_age_limit_ms_{100.0};
  std::atomic<int> offline_feedback_{-1};
  double ready_hold_seconds_{0.05};
  std::int64_t ready_since_ns_{-1};
  bool state_sources_qualified_{false};
  // Set by the motion controller's latched fault_stop topic; consumed by the worker.
  std::atomic_bool motion_fault_request_{false};
  std::atomic_bool motion_fault_pending_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_fault_subscription_;
  std::vector<std::uint8_t> reset_targets_;
  std::vector<hw_semantic_components::Cia402Axis> cia402_axes_;
  std::vector<std::unique_ptr<HandshakeDevice>> handshake_devices_;
  bool device_stage_{false};
  // Extra state values copied into diagnostics: <diagnostic_joints>/<diagnostic_joint_interfaces>.
  std::vector<std::string> diagnostic_state_names_;
  std::vector<std::size_t> diagnostic_state_indices_;
  std::unique_ptr<std::atomic<double>[]> diagnostic_state_values_;
  std::size_t current_batch_{0U};
  std::uint8_t downward_stage_{0U};
  std::int64_t stage_deadline_ns_{0};
  bool enable_preempt_requested_{false};
  Owner interrupted_owner_{Owner::kNone};
  std::atomic_bool jtc_deactivate_failed_{false};
  runtime_log::ShmRingWriter trace_;
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
  bool enable_only_{false};  // Derived from the configured controller set.
  std::vector<std::string> managed_controllers_;
  std::vector<std::string> companion_controllers_;
  std::array<MotionGroup, kMaxMotionGroups> motion_groups_{};
  std::size_t motion_group_count_{0U};
  // Managed axes whose drive mode a group's mode switch changes (any group).
  std::vector<std::uint8_t> drive_switch_axis_;
  std::size_t motion_age_state_index_{0U};

  rclcpp::CallbackGroup::SharedPtr enable_callback_group_;
  rclcpp::CallbackGroup::SharedPtr disable_callback_group_;
  rclcpp::CallbackGroup::SharedPtr reset_callback_group_;
  rclcpp::CallbackGroup::SharedPtr worker_callback_group_;
  rclcpp::Service<robot_rt_control_interfaces::srv::RtEnable>::SharedPtr enable_service_;
  rclcpp::Service<robot_rt_control_interfaces::srv::RtEnable>::SharedPtr disable_service_;
  rclcpp::Service<robot_rt_control_interfaces::srv::RtEnable>::SharedPtr reset_service_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr worker_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr topology_parameter_callback_;
};

}  // namespace ctrl_enable_manager

#endif  // CTRL_ENABLE_MANAGER__ENABLE_MANAGER_CONTROLLER_HPP_
