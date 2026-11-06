#include "ctrl_rolling_trajectory/rolling_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "ctrl_rolling_trajectory/cubic_hermite.hpp"

namespace ctrl_rolling_trajectory
{
namespace
{

JointPoint copyPoint(const PointView & input) noexcept
{
  JointPoint point;
  point.time_ns = input.time_ns;
  std::copy_n(input.positions, kAxisCount, point.positions.begin());
  std::copy_n(input.velocities, kAxisCount, point.velocities.begin());
  return point;
}

bool sampleSegment(
  const JointPoint & start, const JointPoint & end, std::uint64_t time_ns,
  JointPoint & point) noexcept
{
  const auto duration_ns = end.time_ns - start.time_ns;
  const double duration = static_cast<double>(duration_ns) * 1.0e-9;
  const double phase =
    static_cast<double>(time_ns - start.time_ns) / static_cast<double>(duration_ns);
  point.time_ns = time_ns;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    CubicHermite segment;
    ScalarKinematicState state;
    if (
      !buildCubicHermite(
        start.positions[axis], start.velocities[axis], end.positions[axis], end.velocities[axis],
        duration, segment) ||
      !sampleCubicHermite(segment, phase, state)) {
      return false;
    }
    point.positions[axis] = state.position;
    point.velocities[axis] = state.velocity;
  }
  return true;
}

}  // namespace

bool RollingBuffer::configure(
  const BufferConfiguration & configuration, const DynamicEnvelope & envelope) noexcept
{
  if (
    configuration.capacity < 2U || configuration.capacity > kTransportMaxPoints ||
    configuration.max_horizon_ns == 0U) {
    return false;
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (
      !std::isfinite(configuration.splice_position_tolerance[axis]) ||
      !std::isfinite(configuration.splice_velocity_tolerance[axis]) ||
      configuration.splice_position_tolerance[axis] < 0.0 ||
      configuration.splice_velocity_tolerance[axis] < 0.0) {
      return false;
    }
  }
  if (!limit_checker_.configure(envelope)) {
    return false;
  }
  configuration_ = configuration;
  clear();
  return true;
}

void RollingBuffer::clear() noexcept
{
  image_ = TrajectoryImage{};
  anchor_ = JointPoint{};
}

bool RollingBuffer::reset(const JointPoint & anchor) noexcept
{
  if (!limit_checker_.configured() || anchor.time_ns != 0U) {
    return false;
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const auto & limits = limit_checker_.envelope().axes[axis];
    if (
      !std::isfinite(anchor.positions[axis]) || anchor.velocities[axis] != 0.0 ||
      anchor.positions[axis] < limits.position_lower + limits.position_margin_lower ||
      anchor.positions[axis] > limits.position_upper - limits.position_margin_upper) {
      return false;
    }
  }
  clear();
  anchor_ = anchor;
  return true;
}

RejectCode RollingBuffer::validateInput(const BatchView & batch) const noexcept
{
  if (batch.points == nullptr || batch.point_count < 2U) {
    return RejectCode::kInvalidShape;
  }
  if (batch.point_count > configuration_.capacity) {
    return RejectCode::kCapacityExceeded;
  }
  for (std::size_t index = 0U; index < batch.point_count; ++index) {
    const auto & point = batch.points[index];
    if (
      point.positions == nullptr || point.velocities == nullptr ||
      point.position_count != kAxisCount || point.velocity_count != kAxisCount) {
      return RejectCode::kInvalidShape;
    }
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      if (!std::isfinite(point.positions[axis]) || !std::isfinite(point.velocities[axis])) {
        return RejectCode::kNonFinite;
      }
    }
    if (index > 0U && point.time_ns <= batch.points[index - 1U].time_ns) {
      return RejectCode::kNonMonotonicTime;
    }
  }
  return batch.points[0].time_ns == batch.replace_from_ns ? RejectCode::kNone
                                                          : RejectCode::kTimeGap;
}

RejectCode RollingBuffer::prepare(
  const BatchView & batch, const AdmissionContext & context,
  TrajectoryImage & candidate) const noexcept
{
  const auto input_result = validateInput(batch);
  if (input_result != RejectCode::kNone) {
    return input_result;
  }
  if (image_.generation == std::numeric_limits<std::uint64_t>::max()) {
    return RejectCode::kSessionNotAccepting;
  }
  JointPoint splice = anchor_;
  std::size_t history_start = 0U;
  std::size_t prefix_count = 0U;
  if (image_.point_count == 0U) {
    if (batch.replace_from_ns != 0U) {
      return RejectCode::kTimeGap;
    }
    if (context.execution_time_ns != 0U) {
      return RejectCode::kLateReplace;
    }
  } else {
    if (batch.replace_from_ns < context.replaceable_from_ns) {
      return RejectCode::kLateReplace;
    }
    if (
      context.execution_time_ns < image_.points[0].time_ns ||
      context.execution_time_ns > image_.points[image_.point_count - 1U].time_ns ||
      !sampleTrajectoryImage(image_, batch.replace_from_ns, splice)) {
      return RejectCode::kTimeGap;
    }
    while (history_start + 1U < image_.point_count &&
           image_.points[history_start + 1U].time_ns <= context.execution_time_ns) {
      ++history_start;
    }
    while (history_start + prefix_count < image_.point_count &&
           image_.points[history_start + prefix_count].time_ns < batch.replace_from_ns) {
      ++prefix_count;
    }
  }
  const auto continuity = checkSpliceContinuity(
    splice, copyPoint(batch.points[0]), configuration_.splice_position_tolerance,
    configuration_.splice_velocity_tolerance);
  if (continuity != RejectCode::kNone) {
    return continuity;
  }
  if (prefix_count + batch.point_count > configuration_.capacity) {
    return RejectCode::kCapacityExceeded;
  }

  candidate.point_count = prefix_count + batch.point_count;
  candidate.generation = image_.generation + 1U;
  candidate.earliest_changed_ns = image_.generation > context.active_generation
                                    ? std::min(image_.earliest_changed_ns, batch.replace_from_ns)
                                    : batch.replace_from_ns;
  std::copy_n(image_.points.begin() + history_start, prefix_count, candidate.points.begin());
  // A continuous splice needs a single knot: the client first point is checked, never applied.
  candidate.points[prefix_count] = splice;
  for (std::size_t index = 1U; index < batch.point_count; ++index) {
    candidate.points[prefix_count + index] = copyPoint(batch.points[index]);
  }
  const auto horizon = validateHorizon(candidate, context);
  if (horizon != RejectCode::kNone) {
    return horizon;
  }
  // The prefix (including its shortened last cubic) is unchanged. Validate only the new suffix.
  for (std::size_t index = prefix_count; index + 1U < candidate.point_count; ++index) {
    SegmentExtrema extrema;
    const auto direct =
      limit_checker_.checkSegment(candidate.points[index], candidate.points[index + 1U], extrema);
    if (direct.code != RejectCode::kNone) {
      return direct.code;
    }
    StoppingEnvelope stopping;
    const auto viable = limit_checker_.checkStoppingViability(extrema, stopping);
    if (viable.code != RejectCode::kNone) {
      return viable.code;
    }
  }
  return RejectCode::kNone;
}

RejectCode RollingBuffer::commit(
  const TrajectoryImage & candidate, const AdmissionContext & context) noexcept
{
  if (candidate.earliest_changed_ns < context.replaceable_from_ns) {
    return RejectCode::kLateReplace;
  }
  const auto horizon = validateHorizon(candidate, context);
  if (horizon != RejectCode::kNone) {
    return horizon;
  }
  copyTrajectoryImageEffective(candidate, image_);
  return RejectCode::kNone;
}

RejectCode RollingBuffer::validateHorizon(
  const TrajectoryImage & candidate, const AdmissionContext & context) const noexcept
{
  if (
    candidate.points[0].time_ns > context.execution_time_ns ||
    candidate.points[candidate.point_count - 1U].time_ns < context.execution_time_ns) {
    return RejectCode::kTimeGap;
  }
  const auto horizon =
    candidate.points[candidate.point_count - 1U].time_ns - context.execution_time_ns;
  if (!context.priming && horizon <= context.minimum_horizon_ns) {
    return RejectCode::kInsufficientHorizon;
  }
  return horizon > configuration_.max_horizon_ns ? RejectCode::kHorizonExceeded : RejectCode::kNone;
}

bool sampleTrajectoryImage(
  const TrajectoryImage & image, std::uint64_t time_ns, JointPoint & point) noexcept
{
  MonotonicTrajectoryCursor cursor;
  return sampleTrajectoryImageMonotonic(image, time_ns, cursor, point);
}

bool sampleTrajectoryImageMonotonic(
  const TrajectoryImage & image, std::uint64_t time_ns, MonotonicTrajectoryCursor & cursor,
  JointPoint & point, std::size_t * segment_advances) noexcept
{
  if (
    image.point_count == 0U || time_ns < image.points[0].time_ns ||
    time_ns > image.points[image.point_count - 1U].time_ns) {
    return false;
  }
  const bool reset = !cursor.initialized || cursor.generation != image.generation ||
                     time_ns < cursor.last_time_ns || cursor.point_index >= image.point_count;
  std::size_t index = reset ? 0U : cursor.point_index;
  std::size_t advances = 0U;
  while (index + 1U < image.point_count && image.points[index + 1U].time_ns <= time_ns) {
    ++index;
    ++advances;
  }
  if (time_ns == image.points[index].time_ns) {
    point = image.points[index];
  } else if (
    index + 1U >= image.point_count ||
    !sampleSegment(image.points[index], image.points[index + 1U], time_ns, point)) {
    return false;
  }
  cursor.point_index = index;
  cursor.generation = image.generation;
  cursor.last_time_ns = time_ns;
  cursor.initialized = true;
  if (segment_advances != nullptr) {
    *segment_advances = advances;
  }
  return true;
}

}  // namespace ctrl_rolling_trajectory
