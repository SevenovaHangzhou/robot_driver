#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace ctrl_rolling_trajectory
{

namespace
{

std::uint64_t splitMix64(std::uint64_t & state) noexcept
{
  state += 0x9e3779b97f4a7c15ULL;
  std::uint64_t value = state;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

class OptionalRtFlagGuard
{
public:
  OptionalRtFlagGuard(std::atomic_flag & flag, bool required) noexcept
  : flag_(flag),
    required_(required),
    owns_(!required || !flag_.test_and_set(std::memory_order_acquire))
  {
  }

  ~OptionalRtFlagGuard() noexcept
  {
    if (required_ && owns_) {
      flag_.clear(std::memory_order_release);
    }
  }

  OptionalRtFlagGuard(const OptionalRtFlagGuard &) = delete;
  OptionalRtFlagGuard & operator=(const OptionalRtFlagGuard &) = delete;

  [[nodiscard]] bool mayProceed() const noexcept { return owns_; }

private:
  std::atomic_flag & flag_;
  bool required_{false};
  bool owns_{false};
};

class RtStateVersionGuard
{
public:
  explicit RtStateVersionGuard(std::atomic<std::uint64_t> & version) noexcept : version_(version)
  {
    version_.fetch_add(1U, std::memory_order_acq_rel);
  }

  ~RtStateVersionGuard() noexcept { version_.fetch_add(1U, std::memory_order_release); }

  RtStateVersionGuard(const RtStateVersionGuard &) = delete;
  RtStateVersionGuard & operator=(const RtStateVersionGuard &) = delete;

private:
  std::atomic<std::uint64_t> & version_;
};

}  // namespace

controller_interface::InterfaceConfiguration
RollingTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(kAxisCount);
  for (const std::string & joint : runtime_configuration_.joints) {
    configuration.names.emplace_back(joint + "/position");
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
RollingTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(kAxisCount + 1U);
  for (const std::string & joint : runtime_configuration_.joints) {
    configuration.names.emplace_back(joint + "/position");
  }
  configuration.names.emplace_back(kFeedbackAgeInterface);
  return configuration;
}

controller_interface::CallbackReturn RollingTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  if (
    !configured_ || command_interfaces_.size() != kAxisCount ||
    state_interfaces_.size() != kAxisCount + 1U) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Rolling controller requires 14 position command/state pairs and feedback age");
    return controller_interface::CallbackReturn::ERROR;
  }

  std::array<double, kAxisCount> source_commands{};
  std::array<double, kAxisCount> actual_positions{};
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const std::string expected_name = runtime_configuration_.joints[axis] + "/position";
    if (
      command_interfaces_[axis].get_name() != expected_name ||
      state_interfaces_[axis].get_name() != expected_name) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unexpected position interface order at axis %zu", axis);
      return controller_interface::CallbackReturn::ERROR;
    }
    const double source_command = command_interfaces_[axis].get_value();
    const double actual_position = state_interfaces_[axis].get_value();
    if (
      !std::isfinite(source_command) || !std::isfinite(actual_position) ||
      std::abs(source_command - actual_position) >
        runtime_configuration_.takeover_tolerances[axis]) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unsafe rolling-controller takeover at axis %zu", axis);
      return controller_interface::CallbackReturn::ERROR;
    }
    source_commands[axis] = source_command;
    actual_positions[axis] = actual_position;
  }
  if (state_interfaces_[kAxisCount].get_name() != kFeedbackAgeInterface) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing shared feedback-age state interface");
    return controller_interface::CallbackReturn::ERROR;
  }
  const double feedback_age_ms = state_interfaces_[kAxisCount].get_value();

  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (buffer_.sessionState() == SessionState::kTerminated && !buffer_.resetTerminated()) {
      return controller_interface::CallbackReturn::ERROR;
    }
    if (buffer_.sessionState() != SessionState::kNone) {
      RCLCPP_ERROR(get_node()->get_logger(), "Rolling session still exists during activation");
      return controller_interface::CallbackReturn::ERROR;
    }
    controller_boot_id_ = generateIdentifier();
    open_cache_ = OpenCacheEntry{};
    clearActiveCloseCache();
    finalize_cache_ = CloseCacheEntry{};
    last_reject_code_ = RejectCode::kNone;
    last_rejected_sequence_ = 0U;
    accepted_count_ = 0U;
    rejected_count_ = 0U;
    superseded_pending_count_ = 0U;
    timeout_count_.store(0U, std::memory_order_release);
    invariant_failure_count_.store(0U, std::memory_order_release);
    last_service_error_ = ServiceResultMessage::NONE;
    state_sequence_ = 0U;
    stop_requested_.store(false, std::memory_order_release);
  }

  hold_positions_ = source_commands;
  rt_active_trajectory_ = TrajectoryImage{};
  rt_sampling_cursor_.reset();
  rt_identity_ = SessionIdentity{};
  rt_desired_ = JointPoint{};
  rt_desired_.positions = source_commands;
  rt_desired_.velocities.fill(0.0);
  rt_stop_trajectory_ = StopTrajectory{};
  if (!rt_stop_trajectory_.configure(
        envelope_, envelope_.source == LimitsSource::kTestOnly,
        envelope_.source == LimitsSource::kProvisional)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset rolling stop trajectory");
    return controller_interface::CallbackReturn::ERROR;
  }
  rt_observed_session_epoch_ = 0U;
  rt_publication_floor_ = 1U;
  rt_consumed_publication_sequence_ = 0U;
  rt_last_accepted_arrival_ns_ = 0U;
  rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
  rt_stop_elapsed_ns_ = 0U;
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kNone), std::memory_order_release);
  stop_reason_.store(static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_release);
  rt_invariant_stage_.store(
    static_cast<std::uint8_t>(RtInvariantStage::kNone), std::memory_order_release);
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  prime_start_time_ns_.store(0U, std::memory_order_release);
  session_epoch_.store(0U, std::memory_order_release);
  session_publication_floor_.store(1U, std::memory_order_release);
  rt_state_version_.store(0U, std::memory_order_release);
  rt_state_session_epoch_.store(0U, std::memory_order_release);
  rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
  has_accepted_update_.store(false, std::memory_order_release);
  initial_handoff_gate_.clear(std::memory_order_release);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    observed_positions_[axis].store(actual_positions[axis], std::memory_order_release);
    desired_positions_[axis].store(source_commands[axis], std::memory_order_release);
    desired_velocities_[axis].store(0.0, std::memory_order_release);
  }
  observed_feedback_age_ms_.store(feedback_age_ms, std::memory_order_release);
  state_publisher_->on_activate();
  active_.store(true, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RollingTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    active_.store(false, std::memory_order_release);
    buffer_.terminateSession();
    controller_boot_id_ = Identifier{};
    open_cache_ = OpenCacheEntry{};
    clearActiveCloseCache();
    finalize_cache_ = CloseCacheEntry{};
    last_mode_result_ = ModeResultMessage{};
    last_mode_result_sequence_ = 0U;
    stop_requested_.store(false, std::memory_order_release);
    stop_reason_.store(
      static_cast<std::uint8_t>(StopReason::kControllerDeactivated), std::memory_order_release);
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kTerminated), std::memory_order_release);
    has_accepted_update_.store(false, std::memory_order_release);
    const std::uint64_t new_epoch = session_epoch_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
    rt_state_session_epoch_.store(new_epoch, std::memory_order_release);
    rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
    const SessionIdentity invalid_identity{};
    const TrajectoryImage invalid_trajectory{};
    (void)snapshot_exchange_.publish(invalid_identity, invalid_trajectory, steadyNowNs());
  }
  (void)publishPublicState();
  state_publisher_->on_deactivate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RollingTrajectoryController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_.load(std::memory_order_acquire)) {
    return controller_interface::return_type::OK;
  }
  RtStateVersionGuard state_version(rt_state_version_);

  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    observed_positions_[axis].store(state_interfaces_[axis].get_value(), std::memory_order_release);
  }
  observed_feedback_age_ms_.store(
    state_interfaces_[kAxisCount].get_value(), std::memory_order_release);

  const std::int64_t period_ns = period.nanoseconds();
  SessionState state = SessionState::kNone;
  bool coherent_session = false;
  constexpr std::size_t kMaximumEpochReadAttempts = 3U;
  for (std::size_t attempt = 0U; attempt < kMaximumEpochReadAttempts; ++attempt) {
    const std::uint64_t epoch_before = session_epoch_.load(std::memory_order_acquire);
    if (epoch_before != rt_observed_session_epoch_ && !resetRtEpoch(epoch_before)) {
      return controller_interface::return_type::ERROR;
    }
    state = static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire));
    const std::uint64_t epoch_after = session_epoch_.load(std::memory_order_acquire);
    if (epoch_before == epoch_after) {
      coherent_session = true;
      break;
    }
  }
  if (!coherent_session) {
    return writeRtDesired() ? controller_interface::return_type::OK
                            : controller_interface::return_type::ERROR;
  }

  bool update_succeeded = true;
  if (state == SessionState::kPriming) {
    if (stop_requested_.load(std::memory_order_acquire)) {
      StopReason reason = static_cast<StopReason>(stop_reason_.load(std::memory_order_acquire));
      if (reason == StopReason::kNone) {
        reason = StopReason::kGracefulClose;
      }
      update_succeeded = beginRtStop(reason);
    } else if (elapsedExceeds(
                 steadyNowNs(), prime_start_time_ns_.load(std::memory_order_acquire),
                 runtime_configuration_.prime_timeout_ns)) {
      update_succeeded = beginRtStop(StopReason::kPrimeTimeout);
    } else {
      OptionalRtFlagGuard handoff(initial_handoff_gate_, true);
      if (handoff.mayProceed()) {
        const SnapshotConsumeResult consumed = consumeLatestRtTrajectory(true);
        if (consumed == SnapshotConsumeResult::kInvalid) {
          update_succeeded = beginRtStop(StopReason::kInternalInvariant);
        }
      }
    }
  } else if (state == SessionState::kRunning) {
    if (stop_requested_.load(std::memory_order_acquire)) {
      StopReason reason = static_cast<StopReason>(stop_reason_.load(std::memory_order_acquire));
      if (reason == StopReason::kNone) {
        reason = StopReason::kGracefulClose;
      }
      update_succeeded = beginRtStop(reason);
    } else {
      const bool initial_handoff = execution_time_ns_.load(std::memory_order_relaxed) == 0U;
      OptionalRtFlagGuard handoff(initial_handoff_gate_, initial_handoff);
      if (!handoff.mayProceed()) {
        update_succeeded = true;
      } else {
        const SnapshotConsumeResult consumed = consumeLatestRtTrajectory(false);
        if (consumed == SnapshotConsumeResult::kInvalid) {
          update_succeeded = beginRtStop(StopReason::kInternalInvariant);
        } else if (!validPeriod(period_ns)) {
          update_succeeded = beginRtStop(StopReason::kClockAnomaly);
        } else if (elapsedExceeds(
                     steadyNowNs(), rt_last_accepted_arrival_ns_,
                     runtime_configuration_.update_timeout_ns)) {
          update_succeeded = beginRtStop(StopReason::kUpdateTimeout);
        } else {
          std::uint64_t stop_duration_ns = 0U;
          const std::uint64_t execution_ns = execution_time_ns_.load(std::memory_order_relaxed);
          const std::uint64_t buffered_until_ns =
            buffered_until_ns_.load(std::memory_order_relaxed);
          if (!calculateStopDurationNs(rt_desired_, stop_duration_ns)) {
            rt_invariant_stage_.store(
              static_cast<std::uint8_t>(RtInvariantStage::kStopDuration),
              std::memory_order_release);
            update_succeeded = beginRtStop(StopReason::kInternalInvariant);
          } else if (!hasSufficientStoppingHorizon(
                       execution_ns, buffered_until_ns, stop_duration_ns,
                       runtime_configuration_.scheduling_guard)) {
            update_succeeded = beginRtStop(StopReason::kLowWater);
          } else {
            std::uint64_t next_execution_ns = 0U;
            JointPoint candidate;
            const std::uint64_t increment_ns = static_cast<std::uint64_t>(period_ns);
            if (
              !addTime(execution_ns, increment_ns, next_execution_ns) ||
              !sampleTrajectoryImageMonotonic(
                rt_active_trajectory_, next_execution_ns, rt_sampling_cursor_, candidate) ||
              !desiredIsValid(candidate)) {
              rt_invariant_stage_.store(
                static_cast<std::uint8_t>(RtInvariantStage::kTrajectorySample),
                std::memory_order_release);
              update_succeeded = beginRtStop(StopReason::kInternalInvariant);
            } else {
              rt_desired_ = candidate;
              execution_time_ns_.store(next_execution_ns, std::memory_order_release);
              update_succeeded = updateRtReplaceableBoundary();
              if (!update_succeeded) {
                rt_invariant_stage_.store(
                  static_cast<std::uint8_t>(RtInvariantStage::kReplaceableBoundary),
                  std::memory_order_release);
                update_succeeded = beginRtStop(StopReason::kInternalInvariant);
              }
            }
          }
        }
      }
    }
  } else if (state == SessionState::kStopping) {
    update_succeeded = advanceRtStop(period_ns);
  } else if (state == SessionState::kHolding || state == SessionState::kNone) {
    rt_desired_.velocities.fill(0.0);
  }

  if (!update_succeeded || !writeRtDesired()) {
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

Identifier RollingTrajectoryController::generateIdentifier() noexcept
{
  static std::atomic<std::uint64_t> counter{0U};
  const auto system_ticks = std::chrono::system_clock::now().time_since_epoch().count();
  const auto steady_ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  std::uint64_t state = static_cast<std::uint64_t>(system_ticks) ^
                        (static_cast<std::uint64_t>(steady_ticks) << 1U) ^
                        counter.fetch_add(1U, std::memory_order_relaxed);
  Identifier identifier{};
  for (std::size_t word_index = 0U; word_index < 2U; ++word_index) {
    const std::uint64_t word = splitMix64(state);
    for (std::size_t byte = 0U; byte < sizeof(word); ++byte) {
      identifier[word_index * sizeof(word) + byte] = static_cast<std::uint8_t>(word >> (byte * 8U));
    }
  }
  identifier[6] = static_cast<std::uint8_t>((identifier[6] & 0x0fU) | 0x40U);
  identifier[8] = static_cast<std::uint8_t>((identifier[8] & 0x3fU) | 0x80U);
  return identifier;
}

std::uint64_t RollingTrajectoryController::steadyNowNs() noexcept
{
  const auto elapsed = std::chrono::steady_clock::now().time_since_epoch();
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
  return nanoseconds < 0 ? 0U : static_cast<std::uint64_t>(nanoseconds);
}

bool RollingTrajectoryController::isZeroIdentifier(const Identifier & identifier) noexcept
{
  return std::all_of(
    identifier.begin(), identifier.end(), [](std::uint8_t value) { return value == 0U; });
}

bool RollingTrajectoryController::elapsedExceeds(
  std::uint64_t now_ns, std::uint64_t start_ns, std::uint64_t limit_ns) noexcept
{
  return now_ns < start_ns || now_ns - start_ns > limit_ns;
}

bool RollingTrajectoryController::validPeriod(std::int64_t period_ns) const noexcept
{
  return period_ns > 0 && static_cast<std::uint64_t>(period_ns) <=
                            runtime_configuration_.maximum_controller_period_ns;
}

bool RollingTrajectoryController::addTime(
  std::uint64_t value_ns, std::uint64_t increment_ns, std::uint64_t & sum_ns) noexcept
{
  if (increment_ns > std::numeric_limits<std::uint64_t>::max() - value_ns) {
    return false;
  }
  sum_ns = value_ns + increment_ns;
  return true;
}

bool RollingTrajectoryController::resetRtEpoch(std::uint64_t epoch) noexcept
{
  const std::uint64_t publication_floor =
    session_publication_floor_.load(std::memory_order_acquire);
  if (publication_floor == 0U) {
    return false;
  }

  JointPoint hold;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    hold.positions[axis] = desired_positions_[axis].load(std::memory_order_acquire);
    hold.velocities[axis] = 0.0;
  }
  if (!desiredIsValid(hold)) {
    return false;
  }

  StopTrajectory stop_trajectory;
  if (!stop_trajectory.configure(
        envelope_, envelope_.source == LimitsSource::kTestOnly,
        envelope_.source == LimitsSource::kProvisional)) {
    return false;
  }

  rt_active_trajectory_ = TrajectoryImage{};
  rt_sampling_cursor_.reset();
  rt_identity_ = SessionIdentity{};
  rt_desired_ = hold;
  rt_stop_trajectory_ = stop_trajectory;
  rt_publication_floor_ = publication_floor;
  rt_consumed_publication_sequence_ = publication_floor - 1U;
  rt_last_accepted_arrival_ns_ = 0U;
  rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
  rt_stop_elapsed_ns_ = 0U;
  rt_invariant_stage_.store(
    static_cast<std::uint8_t>(RtInvariantStage::kNone), std::memory_order_release);
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  has_accepted_update_.store(false, std::memory_order_release);
  rt_observed_session_epoch_ = epoch;
  return true;
}

RollingTrajectoryController::SnapshotConsumeResult
RollingTrajectoryController::consumeLatestRtTrajectory(bool priming) noexcept
{
  SnapshotLease lease;
  if (!snapshot_exchange_.acquire(lease)) {
    return SnapshotConsumeResult::kNoChange;
  }
  const RollingSnapshot & snapshot = *lease;
  if (
    snapshot.publication_sequence < rt_publication_floor_ ||
    snapshot.publication_sequence <= rt_consumed_publication_sequence_) {
    return SnapshotConsumeResult::kNoChange;
  }

  const TrajectoryImage & trajectory = snapshot.trajectory;
  const std::uint64_t active_generation = active_generation_.load(std::memory_order_relaxed);
  if (
    trajectory.point_count == 0U || trajectory.point_count > kTransportMaxPoints ||
    trajectory.generation == 0U || trajectory.generation <= active_generation ||
    !trajectoryImageStructureIsValid(trajectory)) {
    rt_invariant_stage_.store(
      static_cast<std::uint8_t>(RtInvariantStage::kSnapshotShapeOrGeneration),
      std::memory_order_release);
    return SnapshotConsumeResult::kInvalid;
  }

  const std::uint64_t execution_ns = execution_time_ns_.load(std::memory_order_relaxed);
  if (priming) {
    if (execution_ns != 0U || trajectory.points[0].time_ns != 0U) {
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotPrimingTime),
        std::memory_order_release);
      return SnapshotConsumeResult::kInvalid;
    }
  } else {
    if (
      trajectory.points[0].time_ns > execution_ns ||
      trajectory.points[trajectory.point_count - 1U].time_ns < execution_ns) {
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotShapeOrGeneration),
        std::memory_order_release);
      return SnapshotConsumeResult::kInvalid;
    }
    const std::uint64_t replaceable_from_ns = replaceable_from_ns_.load(std::memory_order_acquire);
    const bool safe_zero_time_handoff = execution_ns == 0U && trajectory.earliest_changed_ns == 0U;
    if (
      snapshot.identity.controller_boot_id != rt_identity_.controller_boot_id ||
      snapshot.identity.session_id != rt_identity_.session_id ||
      snapshot.identity.client_instance_id != rt_identity_.client_instance_id) {
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotIdentity), std::memory_order_release);
      return SnapshotConsumeResult::kInvalid;
    }
    if (trajectory.earliest_changed_ns < replaceable_from_ns && !safe_zero_time_handoff) {
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotLateBoundary),
        std::memory_order_release);
      return SnapshotConsumeResult::kInvalid;
    }
  }

  JointPoint candidate;
  MonotonicTrajectoryCursor candidate_cursor;
  std::uint64_t replacement_boundary_ns = 0U;
  if (
    !sampleTrajectoryImageMonotonic(trajectory, execution_ns, candidate_cursor, candidate) ||
    !desiredIsValid(candidate) ||
    !addTime(execution_ns, runtime_configuration_.replace_lead_ns, replacement_boundary_ns)) {
    rt_invariant_stage_.store(
      static_cast<std::uint8_t>(RtInvariantStage::kSnapshotSample), std::memory_order_release);
    return SnapshotConsumeResult::kInvalid;
  }

  if (!copyTrajectoryImageEffective(trajectory, rt_active_trajectory_)) {
    rt_invariant_stage_.store(
      static_cast<std::uint8_t>(RtInvariantStage::kSnapshotShapeOrGeneration),
      std::memory_order_release);
    return SnapshotConsumeResult::kInvalid;
  }
  rt_sampling_cursor_ = candidate_cursor;
  rt_identity_ = snapshot.identity;
  rt_desired_ = candidate;
  rt_consumed_publication_sequence_ = snapshot.publication_sequence;
  rt_last_accepted_arrival_ns_ = snapshot.arrival_time_ns;
  rt_accepted_arrival_time_ns_.store(snapshot.arrival_time_ns, std::memory_order_release);
  active_generation_.store(trajectory.generation, std::memory_order_release);
  buffered_until_ns_.store(
    trajectory.points[trajectory.point_count - 1U].time_ns, std::memory_order_release);
  replaceable_from_ns_.store(replacement_boundary_ns, std::memory_order_release);
  has_accepted_update_.store(true, std::memory_order_release);
  if (priming) {
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kRunning), std::memory_order_release);
  }
  return SnapshotConsumeResult::kAccepted;
}

bool RollingTrajectoryController::beginRtStop(StopReason reason) noexcept
{
  const SessionState state =
    static_cast<SessionState>(rt_session_state_.load(std::memory_order_relaxed));
  if (state == SessionState::kStopping || state == SessionState::kHolding) {
    return true;
  }
  if (state != SessionState::kPriming && state != SessionState::kRunning) {
    return false;
  }

  std::uint8_t expected_reason = static_cast<std::uint8_t>(StopReason::kNone);
  const bool reason_latched = stop_reason_.compare_exchange_strong(
    expected_reason, static_cast<std::uint8_t>(reason), std::memory_order_acq_rel,
    std::memory_order_acquire);
  if (reason_latched) {
    if (reason == StopReason::kPrimeTimeout || reason == StopReason::kUpdateTimeout) {
      timeout_count_.fetch_add(1U, std::memory_order_relaxed);
    } else if (reason == StopReason::kInternalInvariant) {
      invariant_failure_count_.fetch_add(1U, std::memory_order_relaxed);
    }
  }
  if (!rt_stop_trajectory_.begin(rt_desired_)) {
    return false;
  }
  rt_stop_elapsed_ns_ = 0U;
  stop_requested_.store(false, std::memory_order_release);

  if (rt_stop_trajectory_.state() == StopTrajectoryState::kHolding) {
    JointPoint terminal;
    if (!rt_stop_trajectory_.sample(0U, terminal) || !desiredIsValid(terminal)) {
      return false;
    }
    rt_desired_ = terminal;
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kHolding), std::memory_order_release);
  } else {
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kStopping), std::memory_order_release);
  }
  return true;
}

bool RollingTrajectoryController::advanceRtStop(std::int64_t period_ns) noexcept
{
  const StopReason reason = static_cast<StopReason>(stop_reason_.load(std::memory_order_acquire));
  std::uint64_t increment_ns = runtime_configuration_.nominal_controller_period_ns;
  if (reason != StopReason::kClockAnomaly && validPeriod(period_ns)) {
    increment_ns = static_cast<std::uint64_t>(period_ns);
  }

  std::uint64_t next_elapsed_ns = 0U;
  std::uint64_t next_execution_ns = 0U;
  if (
    !addTime(rt_stop_elapsed_ns_, increment_ns, next_elapsed_ns) ||
    !addTime(execution_time_ns_.load(std::memory_order_relaxed), increment_ns, next_execution_ns)) {
    return false;
  }

  JointPoint candidate;
  if (!rt_stop_trajectory_.sample(next_elapsed_ns, candidate) || !desiredIsValid(candidate)) {
    return false;
  }
  rt_desired_ = candidate;
  rt_stop_elapsed_ns_ = next_elapsed_ns;
  execution_time_ns_.store(next_execution_ns, std::memory_order_release);
  if (rt_stop_trajectory_.state() == StopTrajectoryState::kHolding) {
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kHolding), std::memory_order_release);
  }
  return true;
}

bool RollingTrajectoryController::updateRtReplaceableBoundary() noexcept
{
  std::uint64_t boundary_ns = 0U;
  if (!addTime(
        execution_time_ns_.load(std::memory_order_relaxed), runtime_configuration_.replace_lead_ns,
        boundary_ns)) {
    return false;
  }
  replaceable_from_ns_.store(boundary_ns, std::memory_order_release);
  return true;
}

bool RollingTrajectoryController::writeRtDesired() noexcept
{
  if (!desiredIsValid(rt_desired_)) {
    return false;
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    command_interfaces_[axis].set_value(rt_desired_.positions[axis]);
    desired_positions_[axis].store(rt_desired_.positions[axis], std::memory_order_release);
    desired_velocities_[axis].store(rt_desired_.velocities[axis], std::memory_order_release);
  }
  rt_state_session_epoch_.store(rt_observed_session_epoch_, std::memory_order_release);
  rt_accepted_arrival_time_ns_.store(rt_last_accepted_arrival_ns_, std::memory_order_release);
  return true;
}

bool RollingTrajectoryController::desiredIsValid(const JointPoint & desired) const noexcept
{
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const AxisEnvelope & limits = envelope_.axes[axis];
    const double position = desired.positions[axis];
    const double velocity = desired.velocities[axis];
    const double safe_lower = limits.position_lower + limits.position_margin_lower;
    const double safe_upper = limits.position_upper - limits.position_margin_upper;
    if (
      !std::isfinite(position) || !std::isfinite(velocity) || position < safe_lower ||
      position > safe_upper || velocity > limits.velocity_positive ||
      velocity < -limits.velocity_negative) {
      return false;
    }
  }
  return true;
}

bool RollingTrajectoryController::calculateStopDurationNs(
  const JointPoint & desired, std::uint64_t & duration_ns) const noexcept
{
  if (!desiredIsValid(desired)) {
    return false;
  }
  long double duration_seconds = 0.0L;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const double velocity = desired.velocities[axis];
    const double limit = velocity >= 0.0 ? envelope_.axes[axis].stop_acceleration_positive
                                         : envelope_.axes[axis].stop_acceleration_negative;
    const long double axis_duration =
      std::abs(static_cast<long double>(velocity)) / static_cast<long double>(limit);
    duration_seconds = std::max(duration_seconds, axis_duration);
  }
  const long double nanoseconds = std::ceil(duration_seconds * 1.0e9L);
  if (
    !std::isfinite(nanoseconds) || nanoseconds < 0.0L ||
    nanoseconds > static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
    return false;
  }
  duration_ns = static_cast<std::uint64_t>(nanoseconds);
  return true;
}

}  // namespace ctrl_rolling_trajectory

PLUGINLIB_EXPORT_CLASS(
  ctrl_rolling_trajectory::RollingTrajectoryController, controller_interface::ControllerInterface)
