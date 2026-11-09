#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"

#include <algorithm>
#include <cstdint>

namespace ctrl_rolling_trajectory
{

const RollingTrajectoryController::RtStateView &
RollingTrajectoryController::readRtStateView() noexcept
{
  if (const auto * latest = rt_state_exchange_.acquire()) {
    confirmed_rt_state_ = *latest;
  }
  return confirmed_rt_state_;
}

bool RollingTrajectoryController::buildAdmissionContext(AdmissionContext & context) noexcept
{
  context = AdmissionContext{};
  if (
    static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire)) ==
      SessionState::kPriming &&
    active_generation_.load(std::memory_order_acquire) == 0U) {
    context.priming = true;
    return true;
  }

  const auto & view = readRtStateView();
  if (
    view.session_state != SessionState::kRunning ||
    view.session_epoch != session_epoch_.load(std::memory_order_acquire) ||
    static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire)) !=
      SessionState::kRunning ||
    elapsedExceeds(
      steadyNowNs(), view.published_at_ns, runtime_configuration_.maximum_controller_period_ns)) {
    return false;
  }

  std::uint64_t stop_duration_ns = 0U;
  if (!calculateStopDurationNs(view.desired, stop_duration_ns)) {
    return false;
  }
  const auto execution_ns =
    std::max(view.execution_time_ns, execution_time_ns_.load(std::memory_order_acquire));
  // Braking uses the model acceleration limit: stop time grows by at most the elapsed
  // trajectory time. Reserve that growth when RT has advanced past the completed summary.
  if (!addTime(stop_duration_ns, execution_ns - view.execution_time_ns, stop_duration_ns)) {
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
  context.execution_time_ns = execution_ns;
  // A completed summary can lag the update currently in progress. This live monotonic fence
  // prevents commit from replacing a boundary already advanced by that update.
  context.replaceable_from_ns =
    std::max(view.replaceable_from_ns, replaceable_from_ns_.load(std::memory_order_acquire));
  context.minimum_horizon_ns = minimum_horizon_ns;
  // Consumption acknowledges an older replacement even before its summary is published.
  context.active_generation =
    std::max(view.active_generation, active_generation_.load(std::memory_order_acquire));
  return true;
}

bool RollingTrajectoryController::buildPublicState(StateMessage & state)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  const auto & view = readRtStateView();
  // The serialized admission side can acknowledge STARTING before RT observes a new epoch.
  // All execution state and stop reasons otherwise come from one completed RT publication.
  const bool awaiting_rt = view.session_epoch != session_epoch_.load(std::memory_order_acquire);
  const auto session_state = awaiting_rt ? SessionState::kPriming : view.session_state;
  const auto stop_reason = awaiting_rt ? StopReason::kNone : view.stop_reason;
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
