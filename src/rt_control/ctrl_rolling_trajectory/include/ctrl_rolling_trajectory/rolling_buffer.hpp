#ifndef CTRL_ROLLING_TRAJECTORY__ROLLING_BUFFER_HPP_
#define CTRL_ROLLING_TRAJECTORY__ROLLING_BUFFER_HPP_

#include "ctrl_rolling_trajectory/limit_checker.hpp"

namespace ctrl_rolling_trajectory
{

bool sampleTrajectoryImage(
  const TrajectoryImage & image, std::uint64_t time_ns, JointPoint & point) noexcept;

struct MonotonicTrajectoryCursor
{
  std::size_t point_index{0U};
  std::uint64_t generation{0U};
  std::uint64_t last_time_ns{0U};
  bool initialized{false};
  void reset() noexcept { *this = MonotonicTrajectoryCursor{}; }
};

bool sampleTrajectoryImageMonotonic(
  const TrajectoryImage & image, std::uint64_t time_ns, MonotonicTrajectoryCursor & cursor,
  JointPoint & point, std::size_t * segment_advances = nullptr) noexcept;

struct BufferConfiguration
{
  std::size_t capacity{0U};
  std::uint64_t max_horizon_ns{0U};
  std::array<double, kAxisCount> splice_position_tolerance{};
  std::array<double, kAxisCount> splice_velocity_tolerance{};
};

struct AdmissionContext
{
  std::uint64_t execution_time_ns{0U};
  std::uint64_t replaceable_from_ns{0U};
  std::uint64_t minimum_horizon_ns{0U};
  std::uint64_t active_generation{0U};
  bool priming{false};
};

enum class LimitQuantity : std::uint8_t
{
  kNone, kPositionLower, kPositionUpper, kVelocityPositive, kVelocityNegative,
  kAccelerationPositive, kAccelerationNegative, kStoppingLower, kStoppingUpper
};

// Optional non-RT admission evidence. It never changes rejection or acceptance.
struct BufferRejection
{
  RejectCode code{RejectCode::kNone};
  std::size_t axis{kAxisCount};
  std::size_t segment{0U};
  std::uint64_t start_ns{0U};
  std::uint64_t end_ns{0U};
  LimitQuantity quantity{LimitQuantity::kNone};
  double value{0.0};
  double limit{0.0};
};

// Non-realtime, serialized by the controller callback. Owns only the last accepted plan.
// prepare() checks the new suffix; commit() rechecks time after validation has taken place.
class RollingBuffer
{
public:
  bool configure(
    const BufferConfiguration & configuration, const DynamicEnvelope & envelope) noexcept;
  bool reset(const JointPoint & anchor) noexcept;
  void clear() noexcept;
  [[nodiscard]] const TrajectoryImage & image() const noexcept { return image_; }
  RejectCode prepare(
    const BatchView & batch, const AdmissionContext & context,
    TrajectoryImage & candidate, BufferRejection * rejection = nullptr) const noexcept;
  RejectCode commit(const TrajectoryImage & candidate, const AdmissionContext & context) noexcept;

private:
  RejectCode validateInput(const BatchView & batch) const noexcept;
  RejectCode validateHorizon(
    const TrajectoryImage & candidate, const AdmissionContext & context) const noexcept;
  BufferConfiguration configuration_{};
  LimitChecker limit_checker_{};
  TrajectoryImage image_{};
  JointPoint anchor_{};
};

}  // namespace ctrl_rolling_trajectory

#endif  // CTRL_ROLLING_TRAJECTORY__ROLLING_BUFFER_HPP_
