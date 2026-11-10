#ifndef CTRL_ROLLING_TRAJECTORY__LATEST_SNAPSHOT_EXCHANGE_HPP_
#define CTRL_ROLLING_TRAJECTORY__LATEST_SNAPSHOT_EXCHANGE_HPP_

#include <array>
#include <atomic>
#include <cstddef>

namespace ctrl_rolling_trajectory
{

// One serialized producer and one serialized consumer. The unpublished writing slot and
// retained reading slot stay private until an exchange transfers their ownership.
template <typename Snapshot>
class LatestSnapshotExchange
{
public:
  Snapshot & writingSlot() noexcept { return slots_[writing_]; }

  void publish() noexcept
  {
    writing_ = middle_.exchange(writing_ | kDirty, std::memory_order_acq_rel) & kIndexMask;
  }

  // nullptr means no new publication. A retained pointer stays valid until the next
  // successful acquire(), including while the producer fills or publishes other slots.
  const Snapshot * acquire() noexcept
  {
    if ((middle_.load(std::memory_order_acquire) & kDirty) == 0U) {
      return nullptr;
    }
    reading_ = middle_.exchange(reading_, std::memory_order_acq_rel) & kIndexMask;
    return &slots_[reading_];
  }

private:
  static constexpr std::size_t kDirty = 4U;
  static constexpr std::size_t kIndexMask = 3U;
  std::array<Snapshot, 3U> slots_{};
  std::atomic<std::size_t> middle_{1U};
  std::size_t writing_{2U};
  std::size_t reading_{0U};
};

static_assert(std::atomic<std::size_t>::is_always_lock_free);

}  // namespace ctrl_rolling_trajectory

#endif  // CTRL_ROLLING_TRAJECTORY__LATEST_SNAPSHOT_EXCHANGE_HPP_
