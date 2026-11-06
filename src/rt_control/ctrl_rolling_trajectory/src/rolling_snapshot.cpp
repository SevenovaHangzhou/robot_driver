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
  auto & snapshot = slots_[writing_];
  snapshot.session_epoch = session_epoch;
  copyTrajectoryImageEffective(trajectory, snapshot.trajectory);
  snapshot.arrival_time_ns = arrival_time_ns;
  snapshot.publication_sequence = ++publication_sequence_;
  writing_ = middle_.exchange(writing_ | kDirty, std::memory_order_acq_rel) & kIndexMask;
  return true;
}

const RollingSnapshot * RollingSnapshotExchange::acquire() noexcept
{
  if ((middle_.load(std::memory_order_acquire) & kDirty) == 0U) {
    return nullptr;
  }
  reading_ = middle_.exchange(reading_, std::memory_order_acq_rel) & kIndexMask;
  return &slots_[reading_];
}

}  // namespace ctrl_rolling_trajectory
