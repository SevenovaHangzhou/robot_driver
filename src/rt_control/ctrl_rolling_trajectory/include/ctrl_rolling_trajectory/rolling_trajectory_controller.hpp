#ifndef CTRL_ROLLING_TRAJECTORY__ROLLING_TRAJECTORY_CONTROLLER_HPP_
#define CTRL_ROLLING_TRAJECTORY__ROLLING_TRAJECTORY_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "control_msgs/srv/query_trajectory_state.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "robot_motion_interfaces/msg/rolling_joint_target_batch.hpp"
#include "robot_rt_control_interfaces/msg/rolling_joint_control_state.hpp"
#include "ctrl_rolling_trajectory/axis_set.hpp"
#include "ctrl_rolling_trajectory/limit_checker.hpp"
#include "ctrl_rolling_trajectory/rolling_buffer.hpp"
#include "ctrl_rolling_trajectory/rolling_snapshot.hpp"
#include "ctrl_rolling_trajectory/rolling_types.hpp"
#include "ctrl_rolling_trajectory/session_core.hpp"

namespace ctrl_rolling_trajectory
{

class RollingControllerRtTestPeer;

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
  friend class RollingControllerRtTestPeer;

  using BatchMessage = robot_motion_interfaces::msg::RollingJointTargetBatch;
  using StateMessage = robot_rt_control_interfaces::msg::RollingJointControlState;
  using QueryState = control_msgs::srv::QueryTrajectoryState;

  enum class SnapshotConsumeResult : std::uint8_t { kNoChange = 0U, kAccepted = 1U, kInvalid = 2U };

  struct RtStateView
  {
    std::uint64_t session_epoch{0U};
    SessionState session_state{SessionState::kNone};
    StopReason stop_reason{StopReason::kNone};
    std::uint64_t active_generation{0U};
    std::uint64_t execution_time_ns{0U};
    std::uint64_t replaceable_from_ns{0U};
    std::uint64_t published_at_ns{0U};
    JointPoint desired{};
  };

  struct RuntimeConfiguration
  {
    std::size_t buffer_capacity{0U};
    std::uint64_t max_horizon_ns{0U};
    std::uint64_t update_timeout_ns{0U};
    std::uint64_t replace_lead_ns{0U};
    std::uint64_t state_publish_period_ns{0U};
    std::uint64_t nominal_controller_period_ns{0U};
    std::uint64_t maximum_controller_period_ns{0U};
    SchedulingGuard scheduling_guard{};
    double feedback_age_limit_ms{0.0};
    std::array<double, kAxisCount> takeover_tolerances{};
    std::array<double, kAxisCount> splice_position_tolerances{};
    std::array<double, kAxisCount> splice_velocity_tolerances{};
    JointNames joints{};
  };

  static constexpr const char * kFeedbackAgeInterface = "ethercat_domain/process_data_age_ms";

  static std::uint64_t steadyNowNs() noexcept;
  static bool elapsedExceeds(
    std::uint64_t now_ns, std::uint64_t start_ns, std::uint64_t limit_ns) noexcept;
  bool validPeriod(std::int64_t period_ns) const noexcept;
  static bool addTime(
    std::uint64_t value_ns, std::uint64_t increment_ns, std::uint64_t & sum_ns) noexcept;
  bool loadRuntimeConfiguration(std::string & error);

  void publishRejection(const BatchMessage & message);
  void handleBatch(const BatchMessage::SharedPtr message);
  std::uint8_t admitBatch(
    const BatchMessage & message, std::array<PointView, kTransportMaxPoints> & point_views);
  std::uint8_t startSession(std::int64_t start_ns);
  static std::uint8_t publicStopReason(StopReason reason) noexcept;
  // Consumer calls are serialized by callback_mutex_; lifecycle publication excludes update().
  const RtStateView & readRtStateView() noexcept;
  void publishRtStateView() noexcept;
  void queryHeldState(const QueryState::Request & request, QueryState::Response & response);
  bool buildAdmissionContext(AdmissionContext & context, const char ** failure = nullptr) noexcept;
  bool buildPublicState(StateMessage & state);
  bool publishPublicState();
  bool resetRtEpoch(std::uint64_t epoch) noexcept;
  SnapshotConsumeResult consumeLatestRtTrajectory(bool priming) noexcept;
  bool beginRtStop(StopReason reason) noexcept;
  bool advanceRtStop(std::int64_t period_ns) noexcept;
  bool advanceRtFinal(std::int64_t period_ns) noexcept;
  bool updateRtReplaceableBoundary() noexcept;
  bool writeRtDesired() noexcept;
  bool desiredIsValid(const JointPoint & desired) const noexcept;
  bool calculateStopDurationNs(
    const JointPoint & desired, std::uint64_t & duration_ns) const noexcept;

  // Realtime writers and service readers share these; the doubles must stay lock-free.
  static_assert(std::atomic<double>::is_always_lock_free);
  std::array<std::atomic<double>, kAxisCount> observed_positions_{};
  std::array<std::atomic<double>, kAxisCount> desired_positions_{};
  std::atomic<double> observed_feedback_age_ms_{0.0};
  DynamicEnvelope envelope_{};
  RuntimeConfiguration runtime_configuration_{};
  RollingBuffer buffer_{};
  BufferRejection batch_rejection_{};
  const char * admission_failure_{"none"};
  RollingSnapshotExchange snapshot_exchange_{};
  LatestSnapshotExchange<RtStateView> rt_state_exchange_{};
  // Last completed RT publication, owned by the serialized non-RT consumer.
  RtStateView confirmed_rt_state_{};
  // Public state (R-OUT-07), guarded by callback_mutex_.
  std::uint32_t activation_epoch_{0U};
  std::uint64_t last_sequence_{0U};
  // Newest sequence of this epoch; older ones are stale.
  std::uint64_t epoch_sequence_{0U};
  std::uint8_t last_result_{StateMessage::ACCEPTED};
  // Stop reason of the trajectory before the current session, reported until a batch of the
  // current session is accepted.
  std::uint8_t previous_stop_reason_{StateMessage::STOP_NONE};
  // ROS time of session time 0, guarded by callback_mutex_.
  std::int64_t session_origin_ns_{0};
  std::mutex callback_mutex_{};
  std::mutex state_publish_mutex_{};
  rclcpp::CallbackGroup::SharedPtr callback_group_{};
  rclcpp::Subscription<BatchMessage>::SharedPtr batch_subscription_{};
  rclcpp::Service<QueryState>::SharedPtr query_state_service_{};
  rclcpp_lifecycle::LifecyclePublisher<StateMessage>::SharedPtr state_publisher_{};
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    rejection_publisher_{};
  rclcpp::TimerBase::SharedPtr state_timer_{};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_{};
  std::atomic<std::uint8_t> rt_session_state_{static_cast<std::uint8_t>(SessionState::kNone)};
  std::atomic<std::uint8_t> stop_reason_{static_cast<std::uint8_t>(StopReason::kNone)};
  std::atomic<std::uint64_t> execution_time_ns_{0U};
  std::atomic<std::uint64_t> active_generation_{0U};
  std::atomic<std::uint64_t> buffered_until_ns_{0U};
  std::atomic<std::uint64_t> replaceable_from_ns_{0U};
  // ROS time at which a priming session starts following its first batch.
  std::atomic<std::int64_t> prime_start_ns_{0};
  std::atomic<std::uint64_t> session_epoch_{0U};
  std::atomic<std::uint64_t> session_publication_floor_{1U};
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
  const TrajectoryImage * rt_active_trajectory_{nullptr};
  MonotonicTrajectoryCursor rt_sampling_cursor_{};
  JointPoint rt_desired_{};
  StopTrajectory rt_stop_trajectory_{};
};

}  // namespace ctrl_rolling_trajectory

#endif  // CTRL_ROLLING_TRAJECTORY__ROLLING_TRAJECTORY_CONTROLLER_HPP_
