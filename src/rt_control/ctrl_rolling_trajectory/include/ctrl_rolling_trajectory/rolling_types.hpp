#ifndef CTRL_ROLLING_TRAJECTORY__ROLLING_TYPES_HPP_
#define CTRL_ROLLING_TRAJECTORY__ROLLING_TYPES_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace ctrl_rolling_trajectory
{

inline constexpr std::size_t kAxisCount = 14U;
inline constexpr std::size_t kTransportMaxPoints = 256U;

enum class RejectCode : std::uint8_t {
  kNone,
  kInvalidShape,
  kNonFinite,
  kNonMonotonicTime,
  kLateReplace,
  kTimeGap,
  kCapacityExceeded,
  kInsufficientHorizon,
  kPositionDiscontinuity,
  kVelocityDiscontinuity,
  kPositionLimit,
  kVelocityLimit,
  kAccelerationLimit,
  kNotStoppingViable,
  kSessionNotAccepting,
  kHorizonExceeded,
  kFinalNotAtRest
};

struct PointView
{
  std::uint64_t time_ns{0U};
  const double * positions{nullptr};
  std::size_t position_count{0U};
  const double * velocities{nullptr};
  std::size_t velocity_count{0U};
};

// Epoch and sequence are checked once by the ROS callback. The core only sees trajectory data.
struct BatchView
{
  std::uint64_t replace_from_ns{0U};
  const PointView * points{nullptr};
  std::size_t point_count{0U};
  // Last batch of the trajectory: it must end at rest.
  bool final{false};
};

enum class SessionState : std::uint8_t {
  kNone = 0U,
  kPriming = 1U,
  kRunning = 2U,
  kStopping = 3U,
  kHolding = 4U,
  kTerminated = 5U
};

enum class StopReason : std::uint8_t {
  kNone,
  kUpdateTimeout,
  kLowWater,
  kClockAnomaly,
  kInternalInvariant,
  kControllerDeactivated,
  kCompleted
};

struct JointPoint
{
  std::uint64_t time_ns{0U};
  std::array<double, kAxisCount> positions{};
  std::array<double, kAxisCount> velocities{};
};

struct TrajectoryImage
{
  std::array<JointPoint, kTransportMaxPoints> points{};
  std::size_t point_count{0U};
  std::uint64_t generation{0U};
  // Earliest replacement not yet observed by the realtime consumer.
  std::uint64_t earliest_changed_ns{0U};
  // Ends at rest and needs no further batch: run to the last point, then hold.
  bool final{false};
};

inline void copyTrajectoryImageEffective(
  const TrajectoryImage & source, TrajectoryImage & destination) noexcept
{
  std::copy_n(source.points.begin(), source.point_count, destination.points.begin());
  destination.generation = source.generation;
  destination.earliest_changed_ns = source.earliest_changed_ns;
  destination.final = source.final;
  destination.point_count = source.point_count;
}

static_assert(std::is_trivially_copyable_v<JointPoint>);
static_assert(std::is_trivially_copyable_v<TrajectoryImage>);

}  // namespace ctrl_rolling_trajectory

#endif  // CTRL_ROLLING_TRAJECTORY__ROLLING_TYPES_HPP_
