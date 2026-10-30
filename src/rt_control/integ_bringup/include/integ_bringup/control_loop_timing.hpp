// Copyright 2026 Sevenova Hangzhou
// SPDX-License-Identifier: Apache-2.0
//
// Deadline advance of the controller manager update loop with overrun management.
// Same rule as ros2_control Jazzy/rolling `sleep_for_periodic_cycle` (controller_manager
// sleeping_policies.cpp, `overruns.manage`, default true): when the loop is late, skip the missed
// ticks and wake at the next future tick of the original grid instead of running the missed
// cycles back to back. Humble 2.53.x has no such rule; a blocked read()/write() (for example while
// set_component_state holds resources_lock_ during hardware activation) is then followed by a burst
// of catch-up cycles, each of which sends an EtherCAT frame.
#ifndef INTEG_BRINGUP__CONTROL_LOOP_TIMING_HPP_
#define INTEG_BRINGUP__CONTROL_LOOP_TIMING_HPP_

#include <chrono>
#include <cstdint>

namespace integ_bringup
{

struct DeadlineStep
{
  std::chrono::steady_clock::time_point deadline;
  // Ticks of the period grid that were skipped (0 when on time or when overruns are not managed).
  std::uint64_t missed_cycles{0};
};

/// Next wake-up time after a cycle that started at `deadline`. Real-time safe: no allocation,
/// no system call, no logging. `period` must be positive.
inline DeadlineStep next_deadline(
  std::chrono::steady_clock::time_point deadline, std::chrono::steady_clock::time_point now,
  std::chrono::nanoseconds period, bool manage_overruns) noexcept
{
  DeadlineStep step{deadline + period, 0U};
  if (manage_overruns && step.deadline < now) {
    // Smallest k >= 1 with deadline + (k + 1) * period >= now: the first tick not in the past.
    const auto late = now - step.deadline;
    const auto missed =
      static_cast<std::uint64_t>((late.count() + period.count() - 1) / period.count());
    step.deadline += period * static_cast<std::int64_t>(missed);
    step.missed_cycles = missed;
  }
  return step;
}

}  // namespace integ_bringup

#endif  // INTEG_BRINGUP__CONTROL_LOOP_TIMING_HPP_
