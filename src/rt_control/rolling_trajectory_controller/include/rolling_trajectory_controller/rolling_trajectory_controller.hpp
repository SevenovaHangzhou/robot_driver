#ifndef ROLLING_TRAJECTORY_CONTROLLER__ROLLING_TRAJECTORY_CONTROLLER_HPP_
#define ROLLING_TRAJECTORY_CONTROLLER__ROLLING_TRAJECTORY_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "robot_rt_control_interfaces/msg/rolling_joint_control_state.hpp"
#include "robot_rt_control_interfaces/msg/joint_control_mode.hpp"
#include "robot_rt_control_interfaces/msg/rolling_limits_source.hpp"
#include "robot_rt_control_interfaces/msg/rolling_reject_code.hpp"
#include "robot_rt_control_interfaces/msg/rolling_service_result.hpp"
#include "robot_rt_control_interfaces/msg/rolling_session_state.hpp"
#include "robot_rt_control_interfaces/msg/rolling_stop_reason.hpp"
#include "robot_motion_interfaces/msg/rolling_joint_target_batch.hpp"
#include "robot_rt_control_interfaces/srv/close_rolling_joint_session.hpp"
#include "robot_rt_control_interfaces/srv/open_rolling_joint_session.hpp"
#include "rt_control_interfaces/msg/joint_control_mode_result.hpp"
#include "rolling_trajectory_controller/limit_checker.hpp"
#include "rolling_trajectory_controller/rolling_buffer.hpp"
#include "rolling_trajectory_controller/rolling_snapshot.hpp"
#include "rolling_trajectory_controller/rolling_types.hpp"
#include "rolling_trajectory_controller/session_core.hpp"

namespace rolling_trajectory_controller
{

class RollingControllerTestPeer;
class RollingControllerRtTestPeer;
class RollingControllerStateTestPeer;

class RollingTrajectoryController final : public controller_interface::ControllerInterface
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
  friend class RollingControllerTestPeer;
  friend class RollingControllerRtTestPeer;
  friend class RollingControllerStateTestPeer;

  using BatchMessage = robot_motion_interfaces::msg::RollingJointTargetBatch;
  using CloseService = robot_rt_control_interfaces::srv::CloseRollingJointSession;
  using OpenService = robot_rt_control_interfaces::srv::OpenRollingJointSession;
  using StateMessage = robot_rt_control_interfaces::msg::RollingJointControlState;
  using ModeResultMessage = rt_control_interfaces::msg::JointControlModeResult;
  using ControlModeMessage = robot_rt_control_interfaces::msg::JointControlMode;
  using LimitsSourceMessage = robot_rt_control_interfaces::msg::RollingLimitsSource;
  using RejectCodeMessage = robot_rt_control_interfaces::msg::RollingRejectCode;
  using ServiceResultMessage = robot_rt_control_interfaces::msg::RollingServiceResult;
  using SessionStateMessage = robot_rt_control_interfaces::msg::RollingSessionState;
  using StopReasonMessage = robot_rt_control_interfaces::msg::RollingStopReason;

  struct OpenCacheEntry
  {
    bool valid{false};
    OpenService::Request request{};
    OpenService::Response response{};
  };

  struct CloseCacheEntry
  {
    bool valid{false};
    CloseService::Request request{};
    CloseService::Response response{};
  };

  enum class SnapshotConsumeResult : std::uint8_t
  {
    kNoChange = 0U,
    kAccepted = 1U,
    kInvalid = 2U
  };

  enum class RtInvariantStage : std::uint8_t
  {
    kNone = 0U,
    kSnapshotShapeOrGeneration = 1U,
    kSnapshotPrimingTime = 2U,
    kSnapshotIdentity = 3U,
    kSnapshotLateBoundary = 4U,
    kSnapshotSample = 5U,
    kStopDuration = 6U,
    kTrajectorySample = 7U,
    kReplaceableBoundary = 8U,
    kSnapshotPublication = 9U
  };

  struct RtStateView
  {
    std::uint64_t session_epoch{0U};
    SessionState session_state{SessionState::kNone};
    StopReason stop_reason{StopReason::kNone};
    std::uint64_t active_generation{0U};
    std::uint64_t execution_time_ns{0U};
    std::uint64_t replaceable_from_ns{0U};
    std::uint64_t buffered_until_ns{0U};
    std::uint64_t prime_start_time_ns{0U};
    std::uint64_t accepted_arrival_time_ns{0U};
    std::uint64_t timeout_count{0U};
    std::uint64_t invariant_failure_count{0U};
    bool has_accepted_update{false};
    JointPoint desired{};
  };

  struct RuntimeConfiguration
  {
    std::size_t buffer_capacity{0U};
    std::uint64_t max_horizon_ns{0U};
    std::uint64_t required_initial_horizon_ns{0U};
    std::uint64_t update_timeout_ns{0U};
    std::uint64_t replace_lead_ns{0U};
    std::uint64_t state_publish_period_ns{0U};
    std::uint64_t prime_timeout_ns{0U};
    std::uint64_t nominal_controller_period_ns{0U};
    std::uint64_t maximum_controller_period_ns{0U};
    SchedulingGuard scheduling_guard{};
    double open_feedback_age_limit_ms{0.0};
    std::array<double, kAxisCount> takeover_tolerances{};
    std::array<double, kAxisCount> splice_position_tolerances{};
    std::array<double, kAxisCount> splice_velocity_tolerances{};
  };

  static constexpr const char * kFeedbackAgeInterface =
    "ethercat_domain/process_data_age_ms";
  static constexpr std::size_t kCloseCacheCapacity = 8U;
  static constexpr std::uint64_t kCapabilityBits = 0x1fU;

  static Identifier generateIdentifier() noexcept;
  static std::uint64_t steadyNowNs() noexcept;
  static std::uint64_t encodeDouble(double value) noexcept;
  static double decodeDouble(std::uint64_t bits) noexcept;
  static bool isZeroIdentifier(const Identifier & identifier) noexcept;
  static bool sameOpenRequest(
    const OpenService::Request & lhs, const OpenService::Request & rhs) noexcept;
  static bool sameCloseRequest(
    const CloseService::Request & lhs, const CloseService::Request & rhs) noexcept;
  static bool elapsedExceeds(
    std::uint64_t now_ns, std::uint64_t start_ns, std::uint64_t limit_ns) noexcept;
  bool validPeriod(std::int64_t period_ns) const noexcept;
  static bool addTime(
    std::uint64_t value_ns, std::uint64_t increment_ns, std::uint64_t & sum_ns) noexcept;
  bool loadRuntimeConfiguration(std::string & error);

  void handleOpen(
    const std::shared_ptr<OpenService::Request> request,
    std::shared_ptr<OpenService::Response> response);
  void handleClose(
    const std::shared_ptr<CloseService::Request> request,
    std::shared_ptr<CloseService::Response> response);
  void handleUpdate(const BatchMessage::SharedPtr message);
  void handleModeResult(const ModeResultMessage::SharedPtr message);
  void clearActiveCloseCache() noexcept;
  CloseCacheEntry * findActiveCloseCache(const Identifier & request_id) noexcept;
  CloseCacheEntry * allocateActiveCloseCache() noexcept;
  void setOpenError(OpenService::Response & response, std::uint8_t error_code);
  void setCloseError(CloseService::Response & response, std::uint8_t error_code);
  bool synchronizeBufferStateFromRt() noexcept;
  bool readRtStateView(RtStateView & view) const noexcept;
  bool buildAdmissionContext(AdmissionContext & context) const noexcept;
  bool buildPublicState(StateMessage & state);
  bool publishPublicState();
  bool resetRtEpoch(std::uint64_t epoch) noexcept;
  SnapshotConsumeResult consumeLatestRtTrajectory(bool priming) noexcept;
  bool beginRtStop(StopReason reason) noexcept;
  bool advanceRtStop(std::int64_t period_ns) noexcept;
  bool updateRtReplaceableBoundary() noexcept;
  bool writeRtDesired() noexcept;
  bool desiredIsValid(const JointPoint & desired) const noexcept;
  bool calculateStopDurationNs(
    const JointPoint & desired, std::uint64_t & duration_ns) const noexcept;

  std::array<double, kAxisCount> hold_positions_{};
  std::array<std::atomic<std::uint64_t>, kAxisCount> observed_position_bits_{};
  std::array<std::atomic<std::uint64_t>, kAxisCount> desired_position_bits_{};
  std::array<std::atomic<std::uint64_t>, kAxisCount> desired_velocity_bits_{};
  std::atomic<std::uint64_t> feedback_age_bits_{0U};
  DynamicEnvelope envelope_{};
  RuntimeConfiguration runtime_configuration_{};
  RollingBuffer buffer_{};
  RollingSnapshotExchange snapshot_exchange_{};
  Identifier controller_boot_id_{};
  RejectCode last_reject_code_{RejectCode::kNone};
  std::uint64_t last_rejected_sequence_{0U};
  std::uint64_t accepted_count_{0U};
  std::uint64_t rejected_count_{0U};
  std::uint64_t superseded_pending_count_{0U};
  std::uint64_t state_sequence_{0U};
  std::uint64_t last_mode_result_sequence_{0U};
  ModeResultMessage last_mode_result_{};
  std::atomic<std::uint64_t> timeout_count_{0U};
  std::atomic<std::uint64_t> invariant_failure_count_{0U};
  std::uint8_t last_service_error_{0U};
  OpenCacheEntry open_cache_{};
  std::array<CloseCacheEntry, kCloseCacheCapacity> close_cache_{};
  CloseCacheEntry finalize_cache_{};
  std::mutex callback_mutex_{};
  std::mutex state_publish_mutex_{};
  rclcpp::CallbackGroup::SharedPtr callback_group_{};
  rclcpp::Service<OpenService>::SharedPtr open_service_{};
  rclcpp::Service<CloseService>::SharedPtr close_service_{};
  rclcpp::Subscription<BatchMessage>::SharedPtr update_subscription_{};
  rclcpp::Subscription<ModeResultMessage>::SharedPtr mode_result_subscription_{};
  rclcpp_lifecycle::LifecyclePublisher<StateMessage>::SharedPtr state_publisher_{};
  rclcpp::TimerBase::SharedPtr state_timer_{};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_{};
  std::atomic<std::uint8_t> rt_session_state_{
    static_cast<std::uint8_t>(SessionState::kNone)};
  std::atomic<std::uint8_t> stop_reason_{
    static_cast<std::uint8_t>(StopReason::kNone)};
  std::atomic<std::uint8_t> rt_invariant_stage_{
    static_cast<std::uint8_t>(RtInvariantStage::kNone)};
  std::atomic<std::uint64_t> execution_time_ns_{0U};
  std::atomic<std::uint64_t> active_generation_{0U};
  std::atomic<std::uint64_t> buffered_until_ns_{0U};
  std::atomic<std::uint64_t> replaceable_from_ns_{0U};
  std::atomic<std::uint64_t> prime_start_time_ns_{0U};
  std::atomic<std::uint64_t> session_epoch_{0U};
  std::atomic<std::uint64_t> session_publication_floor_{1U};
  std::atomic<std::uint64_t> rt_state_version_{0U};
  std::atomic<std::uint64_t> rt_state_session_epoch_{0U};
  std::atomic<std::uint64_t> rt_accepted_arrival_time_ns_{0U};
  std::atomic_bool has_accepted_update_{false};
  std::atomic_flag initial_handoff_gate_ = ATOMIC_FLAG_INIT;
  std::atomic_bool stop_requested_{false};
  bool configured_{false};
  std::atomic_bool parameters_frozen_{false};
  std::atomic_bool active_{false};
  std::uint64_t rt_observed_session_epoch_{0U};
  std::uint64_t rt_publication_floor_{1U};
  std::uint64_t rt_consumed_publication_sequence_{0U};
  std::uint64_t rt_last_accepted_arrival_ns_{0U};
  std::uint64_t rt_stop_elapsed_ns_{0U};
  TrajectoryImage rt_active_trajectory_{};
  MonotonicTrajectoryCursor rt_sampling_cursor_{};
  SessionIdentity rt_identity_{};
  JointPoint rt_desired_{};
  StopTrajectory rt_stop_trajectory_{};
};

}  // namespace rolling_trajectory_controller

#endif  // ROLLING_TRAJECTORY_CONTROLLER__ROLLING_TRAJECTORY_CONTROLLER_HPP_
