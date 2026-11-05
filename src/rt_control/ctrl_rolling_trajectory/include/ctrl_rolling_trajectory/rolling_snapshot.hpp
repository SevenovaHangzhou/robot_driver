#ifndef CTRL_ROLLING_TRAJECTORY__ROLLING_SNAPSHOT_HPP_
#define CTRL_ROLLING_TRAJECTORY__ROLLING_SNAPSHOT_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "ctrl_rolling_trajectory/rolling_types.hpp"

namespace ctrl_rolling_trajectory
{

struct RollingSnapshot
{
  std::uint64_t session_epoch{0U};
  TrajectoryImage trajectory{};
  std::uint64_t arrival_time_ns{0U};
  std::uint64_t publication_sequence{0U};
};

// One serialized non-RT producer and one RT consumer. Each owns a slot; the third is the
// published slot. Exchanging the middle index transfers ownership with release/acquire ordering.
// The consumer's slot remains immutable until its next successful acquire(), even if the
// producer publishes many times. Lifecycle callbacks must not run concurrently with update().
class RollingSnapshotExchange
{
public:
  bool publish(
    std::uint64_t session_epoch, const TrajectoryImage & trajectory,
    std::uint64_t arrival_time_ns) noexcept;
  // nullptr means no new publication; a returned pointer remains valid until the next acquire().
  const RollingSnapshot * acquire() noexcept;
  // Producer side only, under the same callback serialization as publish().
  std::uint64_t latestPublicationSequence() const noexcept { return publication_sequence_; }

private:
  static constexpr std::size_t kDirty = 4U;
  static constexpr std::size_t kIndexMask = 3U;
  std::array<RollingSnapshot, 3U> slots_{};
  std::atomic<std::size_t> middle_{1U};
  std::size_t writing_{2U};
  std::size_t reading_{0U};
  std::uint64_t publication_sequence_{0U};
};

static_assert(std::atomic<std::size_t>::is_always_lock_free);

}  // namespace ctrl_rolling_trajectory

#endif  // CTRL_ROLLING_TRAJECTORY__ROLLING_SNAPSHOT_HPP_
