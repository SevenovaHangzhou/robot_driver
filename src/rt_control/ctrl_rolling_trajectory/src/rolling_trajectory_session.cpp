#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"

#include <array>
#include <cmath>
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
  const auto now_ns = get_node()->now().nanoseconds();
  const auto replace_from_ns = rclcpp::Time(message.replace_from).nanoseconds();
  if (
    replace_from_ns - now_ns < static_cast<std::int64_t>(runtime_configuration_.replace_lead_ns)) {
    return State::LATE_REPLACE;
  }

  const auto state = static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire));
  std::int64_t offset_ns = 0;
  if (
    state == SessionState::kRunning ||
    (state == SessionState::kPriming && buffer_.image().point_count != 0U)) {
    offset_ns = replace_from_ns - session_origin_ns_;
  } else if (state == SessionState::kStopping || state == SessionState::kTerminated) {
    return State::NOT_ACCEPTING;
  } else {
    if (
      replace_from_ns - now_ns > static_cast<std::int64_t>(runtime_configuration_.max_horizon_ns)) {
      return State::HORIZON_EXCEEDED;
    }
    const auto started = startSession(replace_from_ns);
    if (started != State::ACCEPTED) {
      return started;
    }
  }
  if (offset_ns < 0) {
    return State::LATE_REPLACE;
  }
  for (std::size_t index = 0U; index < point_count; ++index) {
    const auto & point = message.points[index];
    const auto time_ns = offset_ns + rclcpp::Duration(point.time_from_start).nanoseconds();
    if (time_ns < 0) {
      return State::INVALID_BATCH;
    }
    point_views[index] = PointView{
      static_cast<std::uint64_t>(time_ns), point.positions.data(), point.positions.size(),
      point.velocities.data(), point.velocities.size()};
  }
  const BatchView batch{static_cast<std::uint64_t>(offset_ns), point_views.data(), point_count};
  TrajectoryImage candidate;
  AdmissionContext context;
  if (!buildAdmissionContext(context)) {
    return State::NOT_ACCEPTING;
  }
  auto result = buffer_.prepare(batch, context, candidate);
  if (result == RejectCode::kNone) {
    // Validation is non-realtime: the mutable boundary may have advanced in the meantime.
    if (!buildAdmissionContext(context)) {
      return State::NOT_ACCEPTING;
    }
    result = buffer_.commit(candidate, context);
  }
  if (result != RejectCode::kNone) {
    return publicResult(result);
  }
  if (!snapshot_exchange_.publish(
        session_epoch_.load(std::memory_order_acquire), buffer_.image(), steadyNowNs())) {
    std::uint8_t expected = static_cast<std::uint8_t>(StopReason::kNone);
    (void)stop_reason_.compare_exchange_strong(
      expected, static_cast<std::uint8_t>(StopReason::kInternalInvariant),
      std::memory_order_acq_rel, std::memory_order_acquire);
    stop_requested_.store(true, std::memory_order_release);
    return State::NOT_ACCEPTING;
  }
  return State::ACCEPTED;
}

std::uint8_t RollingTrajectoryController::startSession(std::int64_t start_ns)
{
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
  const auto epoch = session_epoch_.load(std::memory_order_acquire);
  const auto publication = snapshot_exchange_.latestPublicationSequence();
  if (
    epoch == std::numeric_limits<std::uint64_t>::max() ||
    publication == std::numeric_limits<std::uint64_t>::max()) {
    return State::NOT_ACCEPTING;
  }
  if (!buffer_.reset(anchor)) {
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
  prime_start_ns_.store(start_ns, std::memory_order_release);
  session_publication_floor_.store(publication + 1U, std::memory_order_release);
  stop_reason_.store(static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_release);
  stop_requested_.store(false, std::memory_order_release);
  session_epoch_.store(epoch + 1U, std::memory_order_release);
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kPriming), std::memory_order_release);
  return State::ACCEPTED;
}

}  // namespace ctrl_rolling_trajectory
