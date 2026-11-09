#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"

#include <cstddef>
#include <cstdint>
#include <chrono>
#include <thread>
#include <algorithm>

#include "hw_semantic_components/feedback_age.hpp"

namespace ctrl_rolling_trajectory
{

void RollingTrajectoryController::queryHeldState(
  const QueryState::Request & request, QueryState::Response & response)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  response = QueryState::Response{};
  response.message = "current stationary command unavailable";
  if (request.time.sec != 0 || request.time.nanosec != 0U) {
    response.message = "only time zero (current stationary command) is supported";
    return;
  }
  RtStateView view;
  const bool empty_priming =
    static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire)) ==
    SessionState::kPriming && buffer_.image().point_count == 0U;
  if (
    !active_.load(std::memory_order_acquire) || !readRtStateView(view) ||
    view.session_epoch != session_epoch_.load(std::memory_order_acquire) ||
    (view.session_state != SessionState::kNone && view.session_state != SessionState::kHolding &&
     !(view.session_state == SessionState::kPriming && empty_priming)) ||
    !std::all_of(view.desired.velocities.begin(), view.desired.velocities.end(),
                 [](double value) { return value == 0.0; }) ||
    !hw_semantic_components::feedback_fresh(
      observed_feedback_age_ms_.load(std::memory_order_acquire),
      runtime_configuration_.feedback_age_limit_ms)) {
    return;
  }
  response.name.assign(runtime_configuration_.joints.begin(), runtime_configuration_.joints.end());
  response.position.assign(view.desired.positions.begin(), view.desired.positions.end());
  response.velocity.assign(kAxisCount, 0.0);
  response.acceleration.assign(kAxisCount, 0.0);
  response.success = true;
  response.message = "current stationary command";
}

bool RollingTrajectoryController::readRtStateView(RtStateView & view, const char ** failure) const noexcept
{
  if (failure != nullptr) { *failure = "snapshot_busy"; }
  // Non-RT only: let a concurrent update finish instead of exhausting adjacent spins.
  // The budget derives from the configured controller period, not the motion timeout.
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::nanoseconds(runtime_configuration_.maximum_controller_period_ns);
  constexpr std::size_t kMaximumReadAttempts = 64U;
  for (std::size_t attempt = 0U; attempt < kMaximumReadAttempts; ++attempt) {
    const std::uint64_t version_before = rt_state_version_.load(std::memory_order_acquire);
    if ((version_before & 1U) == 0U) {
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
      if (version_before == version_after && (version_after & 1U) == 0U) {
        if (candidate.session_state > SessionState::kTerminated || !desiredIsValid(candidate.desired)) {
          if (failure != nullptr) { *failure = "invalid_desired"; }
          return false;
        }
        view = candidate;
        if (failure != nullptr) { *failure = "none"; }
        return true;
      }
    }
    if (std::chrono::steady_clock::now() >= deadline) { break; }
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  return false;
}

bool RollingTrajectoryController::buildAdmissionContext(
  AdmissionContext & context, const char ** failure) const noexcept
{
  context = AdmissionContext{};
  if (failure != nullptr) { *failure = "none"; }
  if (
    static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire)) ==
      SessionState::kPriming &&
    active_generation_.load(std::memory_order_acquire) == 0U) {
    context.priming = true;
    return true;
  }

  RtStateView view;
  if (!readRtStateView(view, failure)) { return false; }
  if (view.session_state != SessionState::kRunning) {
    if (failure != nullptr) { *failure = "state_not_running"; }
    return false;
  }
  if (view.session_epoch != session_epoch_.load(std::memory_order_acquire)) {
    if (failure != nullptr) { *failure = "epoch_changed"; }
    return false;
  }

  std::uint64_t stop_duration_ns = 0U;
  if (!calculateStopDurationNs(view.desired, stop_duration_ns)) {
    if (failure != nullptr) { *failure = "invalid_desired"; }
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
    if (failure != nullptr) { *failure = "time_overflow"; }
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
