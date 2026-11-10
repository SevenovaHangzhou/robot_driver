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
    buffer_.clear();
    ++activation_epoch_;
    epoch_sequence_ = 0U;
    last_sequence_ = 0U;
    last_result_ = StateMessage::ACCEPTED;
    previous_stop_reason_ = StateMessage::STOP_NONE;
    session_origin_ns_ = 0;
    stop_requested_.store(false, std::memory_order_release);
  }

  rt_active_trajectory_ = nullptr;
  rt_sampling_cursor_.reset();
  rt_desired_ = JointPoint{};
  rt_desired_.positions = source_commands;
  rt_desired_.velocities.fill(0.0);
  rt_stop_trajectory_ = StopTrajectory{};
  if (!rt_stop_trajectory_.configure(envelope_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset rolling stop trajectory");
    return controller_interface::CallbackReturn::ERROR;
  }
  rt_observed_session_epoch_ = 0U;
  rt_publication_floor_ = 1U;
  rt_consumed_publication_sequence_ = 0U;
  rt_last_accepted_arrival_ns_ = 0U;
  rt_stop_elapsed_ns_ = 0U;
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kNone), std::memory_order_release);
  stop_reason_.store(static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_release);
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  prime_start_ns_.store(0, std::memory_order_release);
  session_epoch_.store(0U, std::memory_order_release);
  session_publication_floor_.store(1U, std::memory_order_release);
  initial_handoff_gate_.clear(std::memory_order_release);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    observed_positions_[axis].store(actual_positions[axis], std::memory_order_release);
    desired_positions_[axis].store(source_commands[axis], std::memory_order_release);
  }
  observed_feedback_age_ms_.store(feedback_age_ms, std::memory_order_release);
  publishRtStateView();
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
    buffer_.clear();
    stop_requested_.store(false, std::memory_order_release);
    stop_reason_.store(
      static_cast<std::uint8_t>(StopReason::kControllerDeactivated), std::memory_order_release);
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kTerminated), std::memory_order_release);
    rt_observed_session_epoch_ = session_epoch_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
    publishRtStateView();
  }
  (void)publishPublicState();
  state_publisher_->on_deactivate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RollingTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!active_.load(std::memory_order_acquire)) {
    return controller_interface::return_type::OK;
  }

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
    const bool written = writeRtDesired();
    publishRtStateView();
    return written ? controller_interface::return_type::OK
                   : controller_interface::return_type::ERROR;
  }

  bool update_succeeded = true;
  if (state == SessionState::kPriming) {
    if (stop_requested_.load(std::memory_order_acquire)) {
      update_succeeded = beginRtStop(StopReason::kInternalInvariant);
    } else if (time.nanoseconds() >= prime_start_ns_.load(std::memory_order_acquire)) {
      // The first batch starts at its replace_from; until then the arms hold.
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
      update_succeeded = beginRtStop(StopReason::kInternalInvariant);
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
        } else if (rt_active_trajectory_->final) {
          // Ends at rest and was checked to be stoppable throughout: no update is awaited.
          update_succeeded = advanceRtFinal(period_ns);
          if (!update_succeeded) {
            update_succeeded = beginRtStop(StopReason::kInternalInvariant);
          }
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
                *rt_active_trajectory_, next_execution_ns, rt_sampling_cursor_, candidate) ||
              !desiredIsValid(candidate)) {
              update_succeeded = beginRtStop(StopReason::kInternalInvariant);
            } else {
              rt_desired_ = candidate;
              execution_time_ns_.store(next_execution_ns, std::memory_order_release);
              update_succeeded = updateRtReplaceableBoundary();
              if (!update_succeeded) {
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

  const bool written = update_succeeded && writeRtDesired();
  publishRtStateView();
  return written ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

std::uint64_t RollingTrajectoryController::steadyNowNs() noexcept
{
  const auto elapsed = std::chrono::steady_clock::now().time_since_epoch();
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
  return nanoseconds < 0 ? 0U : static_cast<std::uint64_t>(nanoseconds);
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
  if (!stop_trajectory.configure(envelope_)) {
    return false;
  }

  rt_active_trajectory_ = nullptr;
  rt_sampling_cursor_.reset();
  rt_desired_ = hold;
  rt_stop_trajectory_ = stop_trajectory;
  rt_publication_floor_ = publication_floor;
  rt_consumed_publication_sequence_ = publication_floor - 1U;
  rt_last_accepted_arrival_ns_ = 0U;
  rt_stop_elapsed_ns_ = 0U;
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  rt_observed_session_epoch_ = epoch;
  return true;
}

RollingTrajectoryController::SnapshotConsumeResult
RollingTrajectoryController::consumeLatestRtTrajectory(bool priming) noexcept
{
  const auto * latest = snapshot_exchange_.acquire();
  if (latest == nullptr) {
    return SnapshotConsumeResult::kNoChange;
  }
  const RollingSnapshot & snapshot = *latest;
  if (
    snapshot.publication_sequence < rt_publication_floor_ ||
    snapshot.publication_sequence <= rt_consumed_publication_sequence_) {
    return SnapshotConsumeResult::kNoChange;
  }

  const TrajectoryImage & trajectory = snapshot.trajectory;
  const auto active_generation = active_generation_.load(std::memory_order_relaxed);
  if (
    snapshot.session_epoch != rt_observed_session_epoch_ ||
    trajectory.generation <= active_generation) {
    return SnapshotConsumeResult::kInvalid;
  }
  const auto execution_ns = execution_time_ns_.load(std::memory_order_relaxed);
  if (priming) {
    if (execution_ns != 0U || trajectory.points[0].time_ns != 0U) {
      return SnapshotConsumeResult::kInvalid;
    }
  } else {
    if (
      trajectory.points[0].time_ns > execution_ns ||
      trajectory.points[trajectory.point_count - 1U].time_ns < execution_ns) {
      return SnapshotConsumeResult::kInvalid;
    }
    const auto boundary = replaceable_from_ns_.load(std::memory_order_acquire);
    const bool initial_handoff = execution_ns == 0U && trajectory.earliest_changed_ns == 0U;
    if (trajectory.earliest_changed_ns < boundary && !initial_handoff) {
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
    return SnapshotConsumeResult::kInvalid;
  }

  rt_active_trajectory_ = &trajectory;
  rt_sampling_cursor_ = candidate_cursor;
  rt_desired_ = candidate;
  rt_consumed_publication_sequence_ = snapshot.publication_sequence;
  rt_last_accepted_arrival_ns_ = snapshot.arrival_time_ns;
  active_generation_.store(trajectory.generation, std::memory_order_release);
  buffered_until_ns_.store(
    trajectory.points[trajectory.point_count - 1U].time_ns, std::memory_order_release);
  replaceable_from_ns_.store(replacement_boundary_ns, std::memory_order_release);
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
  (void)stop_reason_.compare_exchange_strong(
    expected_reason, static_cast<std::uint8_t>(reason), std::memory_order_acq_rel,
    std::memory_order_acquire);
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

bool RollingTrajectoryController::advanceRtFinal(std::int64_t period_ns) noexcept
{
  const std::uint64_t end_ns = buffered_until_ns_.load(std::memory_order_relaxed);
  std::uint64_t next_execution_ns = 0U;
  if (!addTime(
        execution_time_ns_.load(std::memory_order_relaxed), static_cast<std::uint64_t>(period_ns),
        next_execution_ns)) {
    return false;
  }
  const bool completed = next_execution_ns >= end_ns;
  if (completed) {
    next_execution_ns = end_ns;
  }
  JointPoint candidate;
  if (
    !sampleTrajectoryImageMonotonic(
      *rt_active_trajectory_, next_execution_ns, rt_sampling_cursor_, candidate) ||
    !desiredIsValid(candidate)) {
    return false;
  }
  rt_desired_ = candidate;
  execution_time_ns_.store(next_execution_ns, std::memory_order_release);
  if (!completed) {
    return updateRtReplaceableBoundary();
  }
  // The last point is at rest (checked on admission): hold it without a stop trajectory.
  std::uint8_t expected_reason = static_cast<std::uint8_t>(StopReason::kNone);
  (void)stop_reason_.compare_exchange_strong(
    expected_reason, static_cast<std::uint8_t>(StopReason::kCompleted), std::memory_order_acq_rel,
    std::memory_order_acquire);
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kHolding), std::memory_order_release);
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
  }
  return true;
}

void RollingTrajectoryController::publishRtStateView() noexcept
{
  const auto state = static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire));
  // Session startup may reset shared fields while holding. Skip publication if its non-RT
  // writer owns the handoff; running/stopping state has no concurrent session reset.
  const bool require_handoff = active_.load(std::memory_order_acquire) &&
                               state != SessionState::kRunning && state != SessionState::kStopping;
  OptionalRtFlagGuard handoff(initial_handoff_gate_, require_handoff);
  if (
    !handoff.mayProceed() ||
    session_epoch_.load(std::memory_order_acquire) != rt_observed_session_epoch_) {
    return;
  }
  auto & view = rt_state_exchange_.writingSlot();
  view.session_epoch = rt_observed_session_epoch_;
  view.session_state = static_cast<SessionState>(rt_session_state_.load(std::memory_order_relaxed));
  view.stop_reason = static_cast<StopReason>(stop_reason_.load(std::memory_order_relaxed));
  view.active_generation = active_generation_.load(std::memory_order_relaxed);
  view.execution_time_ns = execution_time_ns_.load(std::memory_order_relaxed);
  view.replaceable_from_ns = replaceable_from_ns_.load(std::memory_order_relaxed);
  view.desired = rt_desired_;
  view.desired.time_ns = view.execution_time_ns;
  view.published_at_ns = steadyNowNs();
  rt_state_exchange_.publish();
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
