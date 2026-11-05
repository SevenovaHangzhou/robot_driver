#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"

#include <cstddef>
#include <cstdint>

namespace ctrl_rolling_trajectory
{

bool RollingTrajectoryController::synchronizeBufferStateFromRt() noexcept
{
  const SessionState rt_state =
    static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire));
  SessionState buffer_state = buffer_.sessionState();

  if (
    rt_state == SessionState::kRunning &&
    (buffer_state == SessionState::kPriming || buffer_state == SessionState::kRunning)) {
    const std::uint64_t active_generation = active_generation_.load(std::memory_order_acquire);
    if (!buffer_.acknowledgeActiveGeneration(active_generation)) {
      return false;
    }
    buffer_state = buffer_.sessionState();
  }
  if (
    (rt_state == SessionState::kStopping || rt_state == SessionState::kHolding) &&
    (buffer_state == SessionState::kPriming || buffer_state == SessionState::kRunning)) {
    if (!buffer_.requestStop()) {
      return false;
    }
    buffer_state = buffer_.sessionState();
  }
  if (rt_state == SessionState::kHolding && buffer_state == SessionState::kStopping) {
    if (!buffer_.markHolding()) {
      return false;
    }
    buffer_state = buffer_.sessionState();
  }
  if (rt_state == SessionState::kTerminated) {
    if (buffer_state != SessionState::kTerminated) {
      buffer_.terminateSession();
    }
    return true;
  }

  if (rt_state == SessionState::kRunning) {
    return buffer_state == SessionState::kRunning;
  }
  if (rt_state == SessionState::kStopping) {
    return buffer_state == SessionState::kStopping;
  }
  if (rt_state == SessionState::kHolding) {
    return buffer_state == SessionState::kHolding;
  }
  return true;
}

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
    candidate.stop_reason = static_cast<StopReason>(stop_reason_.load(std::memory_order_acquire));
    candidate.active_generation = active_generation_.load(std::memory_order_acquire);
    candidate.execution_time_ns = execution_time_ns_.load(std::memory_order_acquire);
    candidate.replaceable_from_ns = replaceable_from_ns_.load(std::memory_order_acquire);
    candidate.buffered_until_ns = buffered_until_ns_.load(std::memory_order_acquire);
    candidate.prime_start_time_ns = prime_start_time_ns_.load(std::memory_order_acquire);
    candidate.accepted_arrival_time_ns =
      rt_accepted_arrival_time_ns_.load(std::memory_order_acquire);
    candidate.timeout_count = timeout_count_.load(std::memory_order_acquire);
    candidate.invariant_failure_count = invariant_failure_count_.load(std::memory_order_acquire);
    candidate.has_accepted_update = has_accepted_update_.load(std::memory_order_acquire);
    candidate.desired.time_ns = candidate.execution_time_ns;
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      candidate.desired.positions[axis] = desired_positions_[axis].load(std::memory_order_acquire);
      candidate.desired.velocities[axis] =
        desired_velocities_[axis].load(std::memory_order_acquire);
    }

    const std::uint64_t version_after = rt_state_version_.load(std::memory_order_acquire);
    if (
      version_before == version_after && (version_after & 1U) == 0U &&
      candidate.session_state <= SessionState::kTerminated &&
      candidate.stop_reason <= StopReason::kControllerRestart &&
      desiredIsValid(candidate.desired)) {
      view = candidate;
      return true;
    }
  }
  return false;
}

bool RollingTrajectoryController::buildAdmissionContext(AdmissionContext & context) const noexcept
{
  if (
    buffer_.sessionState() == SessionState::kPriming &&
    static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire)) ==
      SessionState::kPriming &&
    active_generation_.load(std::memory_order_acquire) == 0U) {
    context.execution_time_ns = 0U;
    context.replaceable_from_ns = 0U;
    context.minimum_horizon_ns =
      runtime_configuration_.scheduling_guard.one_cycle_detection_ns +
      runtime_configuration_.scheduling_guard.stop_time_growth_ns +
      runtime_configuration_.scheduling_guard.non_rt_to_rt_visibility_ns +
      runtime_configuration_.scheduling_guard.period_quantization_ns;
    return true;
  }

  RtStateView view;
  if (
    !readRtStateView(view) ||
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
  return true;
}

bool RollingTrajectoryController::buildPublicState(StateMessage & state)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  RtStateView view;
  bool coherent = false;
  constexpr std::size_t kMaximumStateBuildAttempts = 8U;
  for (std::size_t attempt = 0U; attempt < kMaximumStateBuildAttempts; ++attempt) {
    if (!synchronizeBufferStateFromRt() || !readRtStateView(view)) {
      continue;
    }
    const std::uint64_t session_epoch = session_epoch_.load(std::memory_order_acquire);
    if (view.session_epoch != session_epoch) {
      continue;
    }

    const SessionState buffer_state = buffer_.sessionState();
    const bool state_aligned =
      (view.session_state == SessionState::kRunning && buffer_state == SessionState::kRunning) ||
      (view.session_state == SessionState::kStopping && buffer_state == SessionState::kStopping) ||
      (view.session_state == SessionState::kHolding && buffer_state == SessionState::kHolding) ||
      (view.session_state == SessionState::kTerminated &&
       buffer_state == SessionState::kTerminated) ||
      (view.session_state == SessionState::kPriming && buffer_state == SessionState::kPriming) ||
      (view.session_state == SessionState::kNone && buffer_state == SessionState::kNone);
    if (!state_aligned) {
      continue;
    }
    if (
      view.session_state == SessionState::kRunning && buffer_.hasPending() &&
      buffer_.pendingImage().generation <= view.active_generation) {
      continue;
    }
    coherent = true;
    break;
  }
  if (!coherent) {
    return false;
  }

  const std::uint64_t now_ns = steadyNowNs();
  const SessionState buffer_state = buffer_.sessionState();
  const bool has_session =
    buffer_state != SessionState::kNone && buffer_state != SessionState::kTerminated;
  const bool pending_valid = buffer_.hasPending();
  const TrajectoryImage & validation_head =
    pending_valid ? buffer_.pendingImage() : buffer_.image();

  state = StateMessage{};
  state.published_at = get_node()->now();
  state.protocol_major = kProtocolMajor;
  state.protocol_minor = kProtocolMinor;
  state.controller_boot_id.uuid = controller_boot_id_;
  if (has_session) {
    state.session_id.uuid = buffer_.identity().session_id;
    state.client_instance_id.uuid = buffer_.identity().client_instance_id;
  }
  state.control_mode.value = active_.load(std::memory_order_acquire)
                               ? ControlModeMessage::ROLLING_READY
                               : ControlModeMessage::DISABLED;
  state.session_state.value = static_cast<std::uint8_t>(view.session_state);
  state.has_session = has_session;
  state.has_accepted_update = view.has_accepted_update;
  state.pending_generation_valid = pending_valid;
  state.test_only_limits = envelope_.source == LimitsSource::kTestOnly;
  state.limits_source.value = static_cast<std::uint8_t>(envelope_.source);
  state.axis_set_hash = axis_set_hash_;
  state.limits_version = envelope_.limits_version;
  state.active_generation = view.active_generation;
  state.validation_base_generation = buffer_.validationBaseGeneration();
  state.pending_generation = pending_valid ? validation_head.generation : 0U;
  state.last_seen_sequence = buffer_.lastSeenSequence();
  state.last_accepted_sequence = buffer_.lastAcceptedSequence();
  state.last_rejected_sequence = last_rejected_sequence_;
  state.execution_time_ns = view.execution_time_ns;
  state.replaceable_from_ns = view.replaceable_from_ns;
  state.buffered_until_ns = view.buffered_until_ns;
  state.available_horizon_ns = view.buffered_until_ns >= view.execution_time_ns
                                 ? view.buffered_until_ns - view.execution_time_ns
                                 : 0U;
  state.prime_age_ns =
    view.session_state == SessionState::kPriming && now_ns >= view.prime_start_time_ns
      ? now_ns - view.prime_start_time_ns
      : 0U;
  state.accepted_update_age_ns = view.has_accepted_update && now_ns >= view.accepted_arrival_time_ns
                                   ? now_ns - view.accepted_arrival_time_ns
                                   : 0U;
  state.buffer_point_count = static_cast<std::uint16_t>(validation_head.point_count);
  state.buffer_capacity = static_cast<std::uint16_t>(buffer_.capacity());
  state.desired_positions = view.desired.positions;
  state.desired_velocities = view.desired.velocities;
  state.last_reject.value = static_cast<std::uint8_t>(last_reject_code_);
  state.stop_reason.value = static_cast<std::uint8_t>(view.stop_reason);
  if (last_mode_result_sequence_ != 0U) {
    state.last_mode_request_id = last_mode_result_.request_id;
    state.last_mode_result = last_mode_result_.result;
    state.source_controller_deactivated = last_mode_result_.source_controller_deactivated;
    state.target_controller_activated = last_mode_result_.target_controller_activated;
    state.restart_required = last_mode_result_.restart_required;
  } else {
    state.last_mode_result.value = ServiceResultMessage::NONE;
    state.source_controller_deactivated = view.stop_reason == StopReason::kControllerDeactivated;
  }
  state.accepted_count = accepted_count_;
  state.rejected_count = rejected_count_;
  state.superseded_pending_count = superseded_pending_count_;
  state.timeout_count = view.timeout_count;
  state.invariant_failure_count = view.invariant_failure_count;
  state.command_deadline_miss_count = 0U;
  state.state_sequence = ++state_sequence_;
  return true;
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
