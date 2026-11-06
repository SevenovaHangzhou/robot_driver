#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"

#include <cstddef>
#include <cstdint>

namespace ctrl_rolling_trajectory
{

bool RollingTrajectoryController::readRtStateView(RtStateView & view) const noexcept
{
  constexpr std::size_t kMaximumReadAttempts = 8U;
  for (std::size_t attempt = 0U; attempt < kMaximumReadAttempts; ++attempt) {
    const std::uint64_t version_before = rt_state_version_.load(std::memory_order_acquire);
    if ((version_before & 1U) != 0U) {
      continue;
    }

    RtStateView candidate;
    candidate.session_epoch = rt_state_session_epoch_.load(std::memory_order_acquire);
    candidate.session_state =
      static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire));
    candidate.active_generation = active_generation_.load(std::memory_order_acquire);
    candidate.execution_time_ns = execution_time_ns_.load(std::memory_order_acquire);
    candidate.replaceable_from_ns = replaceable_from_ns_.load(std::memory_order_acquire);
    candidate.desired.time_ns = candidate.execution_time_ns;
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      candidate.desired.positions[axis] = desired_positions_[axis].load(std::memory_order_acquire);
      candidate.desired.velocities[axis] =
        desired_velocities_[axis].load(std::memory_order_acquire);
    }

    const std::uint64_t version_after = rt_state_version_.load(std::memory_order_acquire);
    if (
      version_before == version_after && (version_after & 1U) == 0U &&
      candidate.session_state <= SessionState::kTerminated && desiredIsValid(candidate.desired)) {
      view = candidate;
      return true;
    }
  }
  return false;
}

bool RollingTrajectoryController::buildAdmissionContext(AdmissionContext & context) const noexcept
{
  context = AdmissionContext{};
  if (
    static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire)) ==
      SessionState::kPriming &&
    active_generation_.load(std::memory_order_acquire) == 0U) {
    context.priming = true;
    return true;
  }

  RtStateView view;
  if (
    !readRtStateView(view) || view.session_state != SessionState::kRunning ||
    view.session_epoch != session_epoch_.load(std::memory_order_acquire)) {
    return false;
  }

  std::uint64_t stop_duration_ns = 0U;
  if (!calculateStopDurationNs(view.desired, stop_duration_ns)) {
    return false;
  }
  std::uint64_t minimum_horizon_ns = stop_duration_ns;
  if (
    !addTime(
      minimum_horizon_ns, runtime_configuration_.scheduling_guard.one_cycle_detection_ns,
      minimum_horizon_ns) ||
    !addTime(
      minimum_horizon_ns, runtime_configuration_.scheduling_guard.stop_time_growth_ns,
      minimum_horizon_ns) ||
    !addTime(
      minimum_horizon_ns, runtime_configuration_.scheduling_guard.non_rt_to_rt_visibility_ns,
      minimum_horizon_ns) ||
    !addTime(
      minimum_horizon_ns, runtime_configuration_.scheduling_guard.period_quantization_ns,
      minimum_horizon_ns)) {
    return false;
  }
  context.execution_time_ns = view.execution_time_ns;
  context.replaceable_from_ns = view.replaceable_from_ns;
  context.minimum_horizon_ns = minimum_horizon_ns;
  context.active_generation = view.active_generation;
  return true;
}

bool RollingTrajectoryController::buildPublicState(StateMessage & state)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  const auto session_state =
    static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire));
  const auto stop_reason = static_cast<StopReason>(stop_reason_.load(std::memory_order_acquire));
  // A session is primed before its first batch is checked; until a batch is accepted it has
  // no trajectory and the previous trajectory's stop reason still applies.
  const bool trajectory_pending =
    session_state == SessionState::kPriming && buffer_.image().point_count == 0U;
  state = StateMessage{};
  state.epoch = activation_epoch_;
  switch (session_state) {
    case SessionState::kPriming:
      state.state = trajectory_pending ? StateMessage::HOLDING : StateMessage::STARTING;
      break;
    case SessionState::kRunning:
      state.state = StateMessage::RUNNING;
      break;
    case SessionState::kStopping:
      state.state = StateMessage::STOPPING;
      break;
    default:
      state.state = StateMessage::HOLDING;
  }
  state.last_sequence = last_sequence_;
  state.last_result = last_result_;
  state.stop_reason = trajectory_pending ? previous_stop_reason_ : publicStopReason(stop_reason);
  return true;
}

std::uint8_t RollingTrajectoryController::publicStopReason(StopReason reason) noexcept
{
  switch (reason) {
    case StopReason::kNone:
      return StateMessage::STOP_NONE;
    case StopReason::kUpdateTimeout:
      return StateMessage::STOP_UPDATE_TIMEOUT;
    case StopReason::kLowWater:
      return StateMessage::STOP_BUFFER_END;
    case StopReason::kControllerDeactivated:
      return StateMessage::STOP_DEACTIVATED;
    case StopReason::kCompleted:
      return StateMessage::STOP_COMPLETED;
    default:
      return StateMessage::STOP_INTERNAL;
  }
}

bool RollingTrajectoryController::publishPublicState()
{
  std::lock_guard<std::mutex> publish_lock(state_publish_mutex_);
  if (!state_publisher_ || !state_publisher_->is_activated()) {
    return false;
  }
  StateMessage state;
  if (!buildPublicState(state)) {
    return false;
  }
  state_publisher_->publish(state);
  return true;
}

}  // namespace ctrl_rolling_trajectory
