#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "ctrl_rolling_trajectory/rolling_snapshot.hpp"

namespace ctrl_rolling_trajectory
{
namespace
{

TrajectoryImage trajectory(std::uint64_t generation)
{
  TrajectoryImage result;
  result.generation = generation;
  result.point_count = 64U;
  for (std::size_t i = 0; i < result.point_count; ++i) {
    result.points[i].time_ns = i;
    result.points[i].positions.fill(static_cast<double>(generation));
    result.points[i].velocities.fill(-static_cast<double>(generation));
  }
  return result;
}

bool coherent(const RollingSnapshot & snapshot)
{
  const auto generation = snapshot.trajectory.generation;
  if (
    snapshot.session_epoch != 42U || snapshot.arrival_time_ns != generation * 10U ||
    snapshot.publication_sequence != generation || snapshot.trajectory.point_count != 64U) {
    return false;
  }
  for (std::size_t i = 0; i < snapshot.trajectory.point_count; ++i) {
    const auto & point = snapshot.trajectory.points[i];
    if (point.time_ns != i) {
      return false;
    }
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      if (
        point.positions[axis] != static_cast<double>(generation) ||
        point.velocities[axis] != -static_cast<double>(generation)) {
        return false;
      }
    }
  }
  return true;
}

TEST(RollingSnapshotExchange, ConsumerGetsTheLatestPublicationAndRetainsItsSlot)
{
  RollingSnapshotExchange exchange;
  EXPECT_EQ(exchange.acquire(), nullptr);
  ASSERT_TRUE(exchange.publish(42U, trajectory(1U), 10U));
  const auto * held = exchange.acquire();
  ASSERT_NE(held, nullptr);
  EXPECT_TRUE(coherent(*held));
  EXPECT_EQ(exchange.acquire(), nullptr);
  for (std::uint64_t generation = 2U; generation <= 100U; ++generation) {
    ASSERT_TRUE(exchange.publish(42U, trajectory(generation), generation * 10U));
    EXPECT_EQ(held->trajectory.generation, 1U);
    EXPECT_TRUE(coherent(*held));
  }
  const auto * latest = exchange.acquire();
  ASSERT_NE(latest, nullptr);
  EXPECT_EQ(latest->trajectory.generation, 100U);
  EXPECT_TRUE(coherent(*latest));
  EXPECT_EQ(exchange.acquire(), nullptr);
}

TEST(RollingSnapshotExchange, MillionPublicationsNeverTearOrOverwriteTheReadingSlot)
{
  RollingSnapshotExchange exchange;
  constexpr std::uint64_t count = 1'000'000U;
  std::atomic_bool done{false};
  std::atomic_bool ok{true};
  std::thread producer([&] {
    for (std::uint64_t generation = 1U; generation <= count; ++generation) {
      if (!exchange.publish(42U, trajectory(generation), generation * 10U)) {
        ok.store(false);
        break;
      }
    }
    done.store(true, std::memory_order_release);
  });
  std::uint64_t previous = 0U;
  while (previous < count && (!done.load(std::memory_order_acquire) || ok.load())) {
    if (const auto * snapshot = exchange.acquire()) {
      const auto generation = snapshot->trajectory.generation;
      if (generation <= previous || !coherent(*snapshot)) {
        ok.store(false);
        break;
      }
      std::this_thread::yield();
      if (snapshot->trajectory.generation != generation || !coherent(*snapshot)) {
        ok.store(false);
        break;
      }
      previous = generation;
    } else {
      std::this_thread::yield();
    }
  }
  producer.join();
  EXPECT_TRUE(ok.load());
  EXPECT_EQ(previous, count);
}

}  // namespace
}  // namespace ctrl_rolling_trajectory
