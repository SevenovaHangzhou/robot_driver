#include "ctrl_rolling_trajectory/rolling_snapshot.hpp"

#include <limits>

namespace ctrl_rolling_trajectory
{

bool RollingSnapshotExchange::publish(
  std::uint64_t session_epoch, const TrajectoryImage & trajectory,
  std::uint64_t arrival_time_ns) noexcept
{
  if (publication_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return false;
  }
  auto & snapshot = slots_.writingSlot();
  snapshot.session_epoch = session_epoch;
  copyTrajectoryImageEffective(trajectory, snapshot.trajectory);
  snapshot.arrival_time_ns = arrival_time_ns;
  snapshot.publication_sequence = ++publication_sequence_;
  slots_.publish();
  return true;
}

const RollingSnapshot * RollingSnapshotExchange::acquire() noexcept
{
  return slots_.acquire();
}

}  // namespace ctrl_rolling_trajectory
