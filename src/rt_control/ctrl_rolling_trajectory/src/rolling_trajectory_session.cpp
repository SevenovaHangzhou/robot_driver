#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <thread>

#include "hw_semantic_components/feedback_age.hpp"

namespace ctrl_rolling_trajectory
{

namespace
{

using State = robot_rt_control_interfaces::msg::RollingJointControlState;

class BlockingFlagGuard
{
public:
  explicit BlockingFlagGuard(std::atomic_flag & flag) noexcept : flag_(flag)
  {
    while (flag_.test_and_set(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
  }

  ~BlockingFlagGuard() noexcept { flag_.clear(std::memory_order_release); }

  BlockingFlagGuard(const BlockingFlagGuard &) = delete;
  BlockingFlagGuard & operator=(const BlockingFlagGuard &) = delete;

private:
  std::atomic_flag & flag_;
};

std::uint8_t publicResult(RejectCode code) noexcept
{
  switch (code) {
    case RejectCode::kNone:
      return State::ACCEPTED;
    case RejectCode::kStaleSequence:
      return State::STALE_SEQUENCE;
    case RejectCode::kLateReplace:
      return State::LATE_REPLACE;
    case RejectCode::kPositionDiscontinuity:
    case RejectCode::kVelocityDiscontinuity:
      return State::SPLICE_MISMATCH;
    case RejectCode::kPositionLimit:
    case RejectCode::kVelocityLimit:
    case RejectCode::kAccelerationLimit:
      return State::LIMIT_EXCEEDED;
    case RejectCode::kNotStoppingViable:
      return State::NOT_STOPPABLE;
    case RejectCode::kCapacityExceeded:
    case RejectCode::kInsufficientHorizon:
    case RejectCode::kHorizonExceeded:
      return State::HORIZON_EXCEEDED;
    case RejectCode::kSessionNotAccepting:
      return State::NOT_ACCEPTING;
    default:
      return State::INVALID_BATCH;
  }
}

}  // namespace

void RollingTrajectoryController::handleBatch(const BatchMessage::SharedPtr message)
{
  if (!message) {
    return;
  }
  BlockingFlagGuard initial_handoff(initial_handoff_gate_);
  std::array<PointView, kTransportMaxPoints> point_views{};
  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (!active_.load(std::memory_order_acquire)) {
    return;
  }
  last_result_ = admitBatch(*message, point_views);
  last_sequence_ = message->sequence;
}

std::uint8_t RollingTrajectoryController::admitBatch(
  const BatchMessage & message, std::array<PointView, kTransportMaxPoints> & point_views)
{
  if (message.epoch != activation_epoch_) {
    return State::WRONG_EPOCH;
  }
  if (message.sequence <= epoch_sequence_) {
    return State::STALE_SEQUENCE;
  }
  epoch_sequence_ = message.sequence;
  const std::size_t point_count = message.points.size();
  if (point_count == 0U || point_count > kTransportMaxPoints) {
    return State::INVALID_BATCH;
  }
  const std::int64_t now_ns = get_node()->now().nanoseconds();
  const std::int64_t replace_from_ns = rclcpp::Time(message.replace_from).nanoseconds();
  if (
    replace_from_ns - now_ns < static_cast<std::int64_t>(runtime_configuration_.replace_lead_ns)) {
    return State::LATE_REPLACE;
  }
  if (!synchronizeBufferStateFromRt()) {
    return State::NOT_ACCEPTING;
  }

  // A batch extends the session once its first batch is accepted (also before it starts);
  // holding (after a stop, or before any accepted batch) the batch starts a new session.
  const SessionState session_state = buffer_.sessionState();
  std::int64_t offset_ns = 0;
  if (
    session_state == SessionState::kRunning ||
    (session_state == SessionState::kPriming && buffer_.hasPending())) {
    offset_ns = replace_from_ns - session_origin_ns_;
  } else if (session_state == SessionState::kStopping) {
    return State::NOT_ACCEPTING;
  } else {
    if (
      replace_from_ns - now_ns > static_cast<std::int64_t>(runtime_configuration_.max_horizon_ns)) {
      return State::HORIZON_EXCEEDED;
    }
    const std::uint8_t started = startSession(replace_from_ns);
    if (started != State::ACCEPTED) {
      return started;
    }
  }

  for (std::size_t index = 0U; index < point_count; ++index) {
    const auto & point = message.points[index];
    const std::int64_t time_ns = offset_ns + rclcpp::Duration(point.time_from_start).nanoseconds();
    if (time_ns < 0) {
      return State::INVALID_BATCH;
    }
    point_views[index] = PointView{
      static_cast<std::uint64_t>(time_ns), point.positions.data(), point.positions.size(),
      point.velocities.data(), point.velocities.size()};
  }
  BatchView batch;
  batch.protocol_major = kProtocolMajor;
  batch.protocol_minor = kProtocolMinor;
  batch.controller_boot_id = buffer_.identity().controller_boot_id;
  batch.session_id = buffer_.identity().session_id;
  batch.client_instance_id = buffer_.identity().client_instance_id;
  batch.sequence = buffer_.lastSeenSequence() + 1U;
  batch.replace_from_ns = static_cast<std::uint64_t>(offset_ns);
  batch.points = point_views.data();
  batch.point_count = point_count;

  PreparedSubmission submission;
  AdmissionContext admission_context;
  if (!buildAdmissionContext(admission_context)) {
    return State::NOT_ACCEPTING;
  }
  RejectCode result = buffer_.prepare(batch, admission_context, submission);
  if (result == RejectCode::kNone) {
    AdmissionContext commit_context;
    if (!synchronizeBufferStateFromRt() || !buildAdmissionContext(commit_context)) {
      return State::NOT_ACCEPTING;
    }
    result = buffer_.commit(submission, commit_context);
  }
  if (result != RejectCode::kNone) {
    return publicResult(result);
  }
  if (!snapshot_exchange_.publish(buffer_.identity(), buffer_.pendingImage(), steadyNowNs())) {
    buffer_.terminateSession();
    std::uint8_t expected_reason = static_cast<std::uint8_t>(StopReason::kNone);
    (void)stop_reason_.compare_exchange_strong(
      expected_reason, static_cast<std::uint8_t>(StopReason::kInternalInvariant),
      std::memory_order_acq_rel, std::memory_order_acquire);
    rt_invariant_stage_.store(
      static_cast<std::uint8_t>(RtInvariantStage::kSnapshotPublication), std::memory_order_release);
    stop_requested_.store(true, std::memory_order_release);
    invariant_failure_count_.fetch_add(1U, std::memory_order_relaxed);
    return State::NOT_ACCEPTING;
  }
  return State::ACCEPTED;
}

std::uint8_t RollingTrajectoryController::startSession(std::int64_t start_ns)
{
  const SessionState session_state = buffer_.sessionState();
  if (session_state == SessionState::kHolding && !buffer_.finishSession()) {
    return State::NOT_ACCEPTING;
  }
  if (session_state == SessionState::kPriming) {
    // A started session whose first batch was rejected.
    buffer_.terminateSession();
    (void)buffer_.resetTerminated();
  }
  if (!hw_semantic_components::feedback_fresh(
        observed_feedback_age_ms_.load(std::memory_order_acquire),
        runtime_configuration_.feedback_age_limit_ms)) {
    return State::NOT_ACCEPTING;
  }

  JointPoint anchor;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const double hold = desired_positions_[axis].load(std::memory_order_acquire);
    const double actual = observed_positions_[axis].load(std::memory_order_acquire);
    if (
      !std::isfinite(actual) ||
      std::abs(hold - actual) > runtime_configuration_.takeover_tolerances[axis]) {
      return State::NOT_ACCEPTING;
    }
    anchor.positions[axis] = hold;
  }
  anchor.velocities.fill(0.0);

  SessionIdentity identity;
  identity.controller_boot_id = controller_boot_id_;
  identity.session_id = generateIdentifier();
  identity.client_instance_id = controller_boot_id_;
  const std::uint64_t current_epoch = session_epoch_.load(std::memory_order_acquire);
  const std::uint64_t latest_publication = snapshot_exchange_.latestPublicationSequence();
  if (
    current_epoch == std::numeric_limits<std::uint64_t>::max() ||
    latest_publication == std::numeric_limits<std::uint64_t>::max()) {
    return State::NOT_ACCEPTING;
  }
  if (!buffer_.beginSession(identity, anchor)) {
    // The held position lies outside the limits minus the margin.
    return State::LIMIT_EXCEEDED;
  }

  const auto stop = static_cast<StopReason>(stop_reason_.load(std::memory_order_acquire));
  if (stop != StopReason::kNone) {
    last_stop_reason_ = publicStopReason(stop);
  }
  session_origin_ns_ = start_ns;
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  has_accepted_update_.store(false, std::memory_order_release);
  prime_start_ns_.store(start_ns, std::memory_order_release);
  session_publication_floor_.store(latest_publication + 1U, std::memory_order_release);
  stop_reason_.store(static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_release);
  stop_requested_.store(false, std::memory_order_release);
  session_epoch_.store(current_epoch + 1U, std::memory_order_release);
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kPriming), std::memory_order_release);
  return State::ACCEPTED;
}

}  // namespace ctrl_rolling_trajectory
