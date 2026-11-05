#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <thread>

#include "ctrl_rolling_trajectory/service_error.hpp"

namespace ctrl_rolling_trajectory
{

namespace
{

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

}  // namespace

bool RollingTrajectoryController::sameOpenRequest(
  const OpenService::Request & lhs, const OpenService::Request & rhs) noexcept
{
  return lhs.protocol_major == rhs.protocol_major && lhs.protocol_minor == rhs.protocol_minor &&
         lhs.request_id.uuid == rhs.request_id.uuid &&
         lhs.expected_controller_boot_id.uuid == rhs.expected_controller_boot_id.uuid &&
         lhs.client_instance_id.uuid == rhs.client_instance_id.uuid &&
         lhs.axis_set_hash == rhs.axis_set_hash;
}

bool RollingTrajectoryController::sameCloseRequest(
  const CloseService::Request & lhs, const CloseService::Request & rhs) noexcept
{
  return lhs.protocol_major == rhs.protocol_major && lhs.protocol_minor == rhs.protocol_minor &&
         lhs.request_id.uuid == rhs.request_id.uuid &&
         lhs.controller_boot_id.uuid == rhs.controller_boot_id.uuid &&
         lhs.session_id.uuid == rhs.session_id.uuid &&
         lhs.client_instance_id.uuid == rhs.client_instance_id.uuid &&
         lhs.operation == rhs.operation;
}

void RollingTrajectoryController::handleOpen(
  const std::shared_ptr<OpenService::Request> request,
  std::shared_ptr<OpenService::Response> response)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  response->request_id = request->request_id;
  response->protocol_major = kProtocolMajor;
  response->protocol_minor = kProtocolMinor;
  response->controller_boot_id.uuid = controller_boot_id_;

  if (request->protocol_major != kProtocolMajor || request->protocol_minor != kProtocolMinor) {
    setOpenError(*response, ServiceResultMessage::WRONG_PROTOCOL);
    return;
  }
  if (isZeroIdentifier(request->request_id.uuid)) {
    setOpenError(*response, ServiceResultMessage::WRONG_REQUEST);
    return;
  }
  if (request->expected_controller_boot_id.uuid != controller_boot_id_) {
    setOpenError(*response, ServiceResultMessage::WRONG_BOOT);
    return;
  }
  if (isZeroIdentifier(request->client_instance_id.uuid)) {
    setOpenError(*response, ServiceResultMessage::WRONG_CLIENT);
    return;
  }
  if (request->axis_set_hash != axis_set_hash_) {
    setOpenError(*response, ServiceResultMessage::AXIS_SET_MISMATCH);
    return;
  }
  if (
    open_cache_.valid && open_cache_.request.request_id.uuid == request->request_id.uuid &&
    open_cache_.request.client_instance_id.uuid == request->client_instance_id.uuid) {
    if (!sameOpenRequest(open_cache_.request, *request)) {
      setOpenError(*response, ServiceResultMessage::WRONG_REQUEST);
      return;
    }
    *response = open_cache_.response;
    last_service_error_ = response->result.value;
    return;
  }
  if (!active_.load(std::memory_order_acquire)) {
    setOpenError(*response, ServiceResultMessage::NOT_READY);
    return;
  }
  if (!synchronizeBufferStateFromRt()) {
    setOpenError(*response, ServiceResultMessage::NOT_READY);
    return;
  }
  if (buffer_.sessionState() != SessionState::kNone) {
    setOpenError(*response, ServiceResultMessage::SESSION_BUSY);
    return;
  }

  const double feedback_age_ms = observed_feedback_age_ms_.load(std::memory_order_acquire);
  if (
    !std::isfinite(feedback_age_ms) || feedback_age_ms < 0.0 ||
    feedback_age_ms > runtime_configuration_.open_feedback_age_limit_ms) {
    setOpenError(*response, ServiceResultMessage::FEEDBACK_STALE);
    return;
  }

  std::array<double, kAxisCount> hold_positions{};
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const double hold = desired_positions_[axis].load(std::memory_order_acquire);
    const double actual = observed_positions_[axis].load(std::memory_order_acquire);
    const AxisEnvelope & limits = envelope_.axes[axis];
    const double safe_lower = limits.position_lower + limits.position_margin_lower;
    const double safe_upper = limits.position_upper - limits.position_margin_upper;
    if (!std::isfinite(hold) || hold < safe_lower || hold > safe_upper) {
      setOpenError(*response, ServiceResultMessage::UNSAFE_HOLD);
      return;
    }
    if (
      !std::isfinite(actual) ||
      std::abs(hold - actual) > runtime_configuration_.takeover_tolerances[axis]) {
      setOpenError(*response, ServiceResultMessage::TAKEOVER_MISMATCH);
      return;
    }
    hold_positions[axis] = hold;
  }

  SessionIdentity identity;
  identity.controller_boot_id = controller_boot_id_;
  identity.session_id = generateIdentifier();
  identity.client_instance_id = request->client_instance_id.uuid;
  JointPoint prime_anchor;
  prime_anchor.positions = hold_positions;
  prime_anchor.velocities.fill(0.0);
  const std::uint64_t current_epoch = session_epoch_.load(std::memory_order_acquire);
  const std::uint64_t latest_publication = snapshot_exchange_.latestPublicationSequence();
  if (
    current_epoch == std::numeric_limits<std::uint64_t>::max() ||
    latest_publication == std::numeric_limits<std::uint64_t>::max()) {
    setOpenError(*response, ServiceResultMessage::RESTART_REQUIRED);
    return;
  }
  if (!buffer_.beginSession(identity, prime_anchor)) {
    setOpenError(*response, ServiceResultMessage::LIMITS_UNAVAILABLE);
    return;
  }

  response->accepted = true;
  response->result.value = ServiceResultMessage::NONE;
  populateServiceError(response->error, ServiceResultMessage::NONE);
  response->client_instance_id = request->client_instance_id;
  response->session_id.uuid = identity.session_id;
  response->session_state.value = SessionStateMessage::PRIMING;
  response->axis_set_hash = axis_set_hash_;
  response->limits_version = envelope_.limits_version;
  response->limits_source.value = static_cast<std::uint8_t>(envelope_.source);
  response->capability_bits = kCapabilityBits;
  response->transport_max_points = static_cast<std::uint16_t>(kTransportMaxPoints);
  response->buffer_capacity = static_cast<std::uint16_t>(buffer_.capacity());
  response->max_horizon_ns = runtime_configuration_.max_horizon_ns;
  response->required_initial_horizon_ns = runtime_configuration_.required_initial_horizon_ns;
  response->replace_lead_ns = runtime_configuration_.replace_lead_ns;
  response->update_timeout_ns = runtime_configuration_.update_timeout_ns;
  response->nominal_controller_period_ns = runtime_configuration_.nominal_controller_period_ns;
  response->initial_replaceable_from_ns = 0U;
  response->test_only_limits = envelope_.source == LimitsSource::kTestOnly;
  response->hold_positions = hold_positions;
  response->hold_velocities.fill(0.0);
  open_cache_.valid = true;
  open_cache_.request = *request;
  open_cache_.response = *response;
  clearActiveCloseCache();
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  has_accepted_update_.store(false, std::memory_order_release);
  prime_start_time_ns_.store(steadyNowNs(), std::memory_order_release);
  session_publication_floor_.store(latest_publication + 1U, std::memory_order_release);
  stop_reason_.store(static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_release);
  stop_requested_.store(false, std::memory_order_release);
  session_epoch_.store(current_epoch + 1U, std::memory_order_release);
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kPriming), std::memory_order_release);
  last_service_error_ = ServiceResultMessage::NONE;
}

void RollingTrajectoryController::handleClose(
  const std::shared_ptr<CloseService::Request> request,
  std::shared_ptr<CloseService::Response> response)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  response->request_id = request->request_id;
  if (request->protocol_major != kProtocolMajor || request->protocol_minor != kProtocolMinor) {
    setCloseError(*response, ServiceResultMessage::WRONG_PROTOCOL);
    return;
  }
  if (
    isZeroIdentifier(request->request_id.uuid) ||
    (request->operation != CloseService::Request::REQUEST_STOP &&
     request->operation != CloseService::Request::FINALIZE)) {
    setCloseError(*response, ServiceResultMessage::WRONG_REQUEST);
    return;
  }
  if (request->controller_boot_id.uuid != controller_boot_id_) {
    setCloseError(*response, ServiceResultMessage::WRONG_BOOT);
    return;
  }
  if (
    finalize_cache_.valid && finalize_cache_.request.request_id.uuid == request->request_id.uuid) {
    if (!sameCloseRequest(finalize_cache_.request, *request)) {
      setCloseError(*response, ServiceResultMessage::WRONG_REQUEST);
      return;
    }
    *response = finalize_cache_.response;
    last_service_error_ = response->result.value;
    return;
  }
  if (!active_.load(std::memory_order_acquire)) {
    setCloseError(*response, ServiceResultMessage::NOT_READY);
    return;
  }
  if (!synchronizeBufferStateFromRt()) {
    setCloseError(*response, ServiceResultMessage::NOT_READY);
    return;
  }

  const SessionState state = buffer_.sessionState();
  if (state == SessionState::kNone || state == SessionState::kTerminated) {
    setCloseError(*response, ServiceResultMessage::WRONG_SESSION);
    return;
  }
  const SessionIdentity & identity = buffer_.identity();
  if (request->session_id.uuid != identity.session_id) {
    setCloseError(*response, ServiceResultMessage::WRONG_SESSION);
    return;
  }
  if (request->client_instance_id.uuid != identity.client_instance_id) {
    setCloseError(*response, ServiceResultMessage::WRONG_CLIENT);
    return;
  }

  if (CloseCacheEntry * cached = findActiveCloseCache(request->request_id.uuid)) {
    if (!sameCloseRequest(cached->request, *request)) {
      setCloseError(*response, ServiceResultMessage::WRONG_REQUEST);
      return;
    }
    *response = cached->response;
    last_service_error_ = response->result.value;
    return;
  }

  if (request->operation == CloseService::Request::FINALIZE) {
    if (state != SessionState::kHolding) {
      setCloseError(*response, ServiceResultMessage::WRONG_MODE);
      return;
    }
    if (!buffer_.finishSession()) {
      setCloseError(*response, ServiceResultMessage::NOT_READY);
      return;
    }
    response->accepted = true;
    response->completed = true;
    response->result.value = ServiceResultMessage::NONE;
    populateServiceError(response->error, ServiceResultMessage::NONE);
    response->session_state.value = SessionStateMessage::NONE;
    response->stop_reason.value = StopReasonMessage::GRACEFUL_CLOSE;
    finalize_cache_.valid = true;
    finalize_cache_.request = *request;
    finalize_cache_.response = *response;
    clearActiveCloseCache();
    stop_requested_.store(false, std::memory_order_release);
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kNone), std::memory_order_release);
    stop_reason_.store(static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_release);
    has_accepted_update_.store(false, std::memory_order_release);
    session_epoch_.fetch_add(1U, std::memory_order_acq_rel);
    last_service_error_ = ServiceResultMessage::NONE;
    return;
  }

  CloseCacheEntry * cache = allocateActiveCloseCache();
  if (cache == nullptr) {
    setCloseError(*response, ServiceResultMessage::WRONG_REQUEST);
    return;
  }
  if (state == SessionState::kPriming || state == SessionState::kRunning) {
    if (!buffer_.requestStop()) {
      setCloseError(*response, ServiceResultMessage::NOT_READY);
      return;
    }
    std::uint8_t expected_reason = static_cast<std::uint8_t>(StopReason::kNone);
    (void)stop_reason_.compare_exchange_strong(
      expected_reason, static_cast<std::uint8_t>(StopReason::kGracefulClose),
      std::memory_order_acq_rel, std::memory_order_acquire);
    stop_requested_.store(true, std::memory_order_release);
  } else if (state != SessionState::kStopping && state != SessionState::kHolding) {
    setCloseError(*response, ServiceResultMessage::NOT_READY);
    return;
  }

  response->accepted = true;
  response->completed = state == SessionState::kHolding;
  response->result.value = ServiceResultMessage::NONE;
  populateServiceError(response->error, ServiceResultMessage::NONE);
  response->session_state.value =
    state == SessionState::kHolding ? SessionStateMessage::HOLDING : SessionStateMessage::STOPPING;
  response->stop_reason.value = StopReasonMessage::GRACEFUL_CLOSE;
  cache->valid = true;
  cache->request = *request;
  cache->response = *response;
  last_service_error_ = ServiceResultMessage::NONE;
}

void RollingTrajectoryController::handleUpdate(const BatchMessage::SharedPtr message)
{
  if (!message) {
    return;
  }
  BlockingFlagGuard initial_handoff(initial_handoff_gate_);
  std::array<PointView, kTransportMaxPoints> point_views{};
  const std::size_t point_count = message->points.size();
  if (point_count <= point_views.size()) {
    for (std::size_t index = 0U; index < point_count; ++index) {
      const auto & point = message->points[index];
      point_views[index] = PointView{
        point.time_from_session_start_ns, point.positions.data(), point.positions.size(),
        point.velocities.data(), point.velocities.size()};
    }
  }

  std::lock_guard<std::mutex> lock(callback_mutex_);
  RejectCode result = RejectCode::kNone;
  PreparedSubmission submission;
  AdmissionContext admission_context;
  bool supersedes_pending = false;
  if (!synchronizeBufferStateFromRt() || !buildAdmissionContext(admission_context)) {
    result = RejectCode::kSessionNotAccepting;
  } else if (
    message->protocol_major != kProtocolMajor || message->protocol_minor != kProtocolMinor) {
    result = RejectCode::kWrongProtocol;
  } else if (message->controller_boot_id.uuid != controller_boot_id_) {
    result = RejectCode::kWrongBoot;
  } else {
    BatchView batch;
    batch.protocol_major = message->protocol_major;
    batch.protocol_minor = message->protocol_minor;
    batch.controller_boot_id = message->controller_boot_id.uuid;
    batch.session_id = message->session_id.uuid;
    batch.client_instance_id = message->client_instance_id.uuid;
    batch.sequence = message->sequence;
    batch.replace_from_ns = message->replace_from_ns;
    batch.points = point_views.data();
    batch.point_count = point_count;
    supersedes_pending = buffer_.hasPending();
    result = buffer_.prepare(batch, admission_context, submission);
    if (result == RejectCode::kNone) {
      AdmissionContext commit_context;
      if (!synchronizeBufferStateFromRt() || !buildAdmissionContext(commit_context)) {
        result = RejectCode::kSessionNotAccepting;
      } else {
        result = buffer_.commit(submission, commit_context);
      }
    }
  }

  if (result == RejectCode::kNone) {
    if (!snapshot_exchange_.publish(buffer_.identity(), buffer_.pendingImage(), steadyNowNs())) {
      buffer_.terminateSession();
      std::uint8_t expected_reason = static_cast<std::uint8_t>(StopReason::kNone);
      (void)stop_reason_.compare_exchange_strong(
        expected_reason, static_cast<std::uint8_t>(StopReason::kInternalInvariant),
        std::memory_order_acq_rel, std::memory_order_acquire);
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotPublication),
        std::memory_order_release);
      stop_requested_.store(true, std::memory_order_release);
      invariant_failure_count_.fetch_add(1U, std::memory_order_relaxed);
      result = RejectCode::kSessionNotAccepting;
    } else {
      ++accepted_count_;
      if (supersedes_pending) {
        ++superseded_pending_count_;
      }
    }
  }
  last_reject_code_ = result;
  if (result != RejectCode::kNone) {
    last_rejected_sequence_ = message->sequence;
    ++rejected_count_;
  }
}

void RollingTrajectoryController::handleModeResult(const ModeResultMessage::SharedPtr message)
{
  if (
    message == nullptr || message->protocol_major != ModeResultMessage::PROTOCOL_MAJOR ||
    message->protocol_minor != ModeResultMessage::PROTOCOL_MINOR || message->sequence == 0U ||
    isZeroIdentifier(message->request_id.uuid) ||
    message->result.value > ServiceResultMessage::NOT_READY ||
    message->mode.value > ControlModeMessage::RESTART_REQUIRED) {
    return;
  }
  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (message->sequence <= last_mode_result_sequence_) {
    return;
  }
  last_mode_result_ = *message;
  last_mode_result_sequence_ = message->sequence;
}

void RollingTrajectoryController::clearActiveCloseCache() noexcept
{
  for (CloseCacheEntry & entry : close_cache_) {
    entry = CloseCacheEntry{};
  }
}

RollingTrajectoryController::CloseCacheEntry * RollingTrajectoryController::findActiveCloseCache(
  const Identifier & request_id) noexcept
{
  for (CloseCacheEntry & entry : close_cache_) {
    if (entry.valid && entry.request.request_id.uuid == request_id) {
      return &entry;
    }
  }
  return nullptr;
}

RollingTrajectoryController::CloseCacheEntry *
RollingTrajectoryController::allocateActiveCloseCache() noexcept
{
  for (CloseCacheEntry & entry : close_cache_) {
    if (!entry.valid) {
      return &entry;
    }
  }
  return nullptr;
}

void RollingTrajectoryController::setOpenError(
  OpenService::Response & response, std::uint8_t error_code)
{
  response.accepted = false;
  response.result.value = error_code;
  populateServiceError(response.error, error_code);
  last_service_error_ = error_code;
}

void RollingTrajectoryController::setCloseError(
  CloseService::Response & response, std::uint8_t error_code)
{
  response.accepted = false;
  response.completed = false;
  response.result.value = error_code;
  populateServiceError(response.error, error_code);
  response.session_state.value = static_cast<std::uint8_t>(buffer_.sessionState());
  response.stop_reason.value = buffer_.sessionState() == SessionState::kStopping ||
                                   buffer_.sessionState() == SessionState::kHolding
                                 ? StopReasonMessage::GRACEFUL_CLOSE
                                 : StopReasonMessage::NONE;
  last_service_error_ = error_code;
}

}  // namespace ctrl_rolling_trajectory
