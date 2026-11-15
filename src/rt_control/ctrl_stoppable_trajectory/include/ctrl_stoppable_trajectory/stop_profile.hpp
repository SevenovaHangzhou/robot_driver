#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

// Receiver-side numerical core only. The caller owns request transport, lifecycle
// and goal completion. All state below is fixed-size and owned by one RT consumer.
namespace execution_stop
{
struct Limits
{
  double velocity, acceleration, jerk;
};
struct State
{
  double position, velocity, acceleration;
};
struct Segment
{
  double duration{}, jerk{};
};
enum class Result { accepted, invalid, unreachable, past_target, stale_goal, inactive };
using GoalId = std::array<std::uint8_t, 16>;

class Profile
{
public:
  Result begin(State initial, Limits limits, double target) noexcept
  {
    if (
      !std::isfinite(initial.position) || !std::isfinite(initial.velocity) ||
      !std::isfinite(initial.acceleration) || !std::isfinite(target) ||
      !positive(limits.velocity) || !positive(limits.acceleration) || !positive(limits.jerk)) {
      return Result::invalid;
    }
    const double v = initial.velocity, a = initial.acceleration;
    const double limit_a = limits.acceleration, j = limits.jerk;
    if (
      v < -kStateRoundoff || std::abs(a) > limit_a + kStateRoundoff ||
      (a < 0. && v < a * a / (2. * j) - kStateRoundoff) ||
      v + std::pow(std::max(a, 0.), 2) / (2. * j) > limits.velocity + kStateRoundoff) {
      return Result::unreachable;
    }
    Profile candidate;
    candidate.initial_ = initial;
    if (std::abs(v) > kStateRoundoff || std::abs(a) > kStateRoundoff) {
      const double peak = std::min(limit_a, std::sqrt(j * std::max(v, 0.) + a * a / 2.));
      candidate.segments_ = {
        {{std::max((a + peak) / j, 0.), -j},
         {std::max((v + (a * a - 2. * peak * peak) / (2. * j)) / peak, 0.), 0.},
         {peak / j, j}}};
    }
    const State terminal = candidate.integrate(candidate.duration());
    if (
      std::abs(terminal.velocity) > kStateRoundoff ||
      std::abs(terminal.acceleration) > kStateRoundoff) {
      return Result::unreachable;
    }
    if (terminal.position > target + kStateRoundoff) {
      return Result::past_target;
    }
    *this = candidate;
    return Result::accepted;
  }

  State integrate(double elapsed) const noexcept
  {
    State state = initial_;
    double remaining = std::clamp(elapsed, 0., duration());
    for (const auto & segment : segments_) {
      const double dt = std::min(remaining, segment.duration);
      state.position +=
        state.velocity * dt + state.acceleration * dt * dt / 2. + segment.jerk * dt * dt * dt / 6.;
      state.velocity += state.acceleration * dt + segment.jerk * dt * dt / 2.;
      state.acceleration += segment.jerk * dt;
      remaining -= dt;
    }
    return state;
  }

  State sample(double elapsed) const noexcept
  {
    State state = integrate(elapsed);
    if (elapsed >= duration()) {
      state.velocity = state.acceleration = 0.;
    }
    return state;
  }

  double duration() const noexcept
  {
    double value = 0.;
    for (const auto & segment : segments_) {
      value += segment.duration;
    }
    return value;
  }

  const std::array<Segment, 3> & segments() const noexcept { return segments_; }

private:
  // Numerical integration budget only; not an actuator or tracking tolerance.
  static constexpr double kStateRoundoff = 1e-10;
  static bool positive(double x) noexcept { return std::isfinite(x) && x > 0.; }
  State initial_{};
  std::array<Segment, 3> segments_{};
};

// Request identity is checked when consumed, not only when received. The caller
// supplies the current reference from the same execution cycle as this call.
class Receiver
{
public:
  Result apply(
    const GoalId & requested, const GoalId & executing, bool active, State reference_now,
    Limits limits, double target, std::int64_t now_ns) noexcept
  {
    if (!active) {
      return Result::inactive;
    }
    if (requested != executing) {
      return Result::stale_goal;
    }
    if (stopping_ && owner_ == executing) {
      return Result::accepted;
    }
    Profile candidate;
    const Result result = candidate.begin(reference_now, limits, target);
    if (result != Result::accepted) {
      return result;
    }
    profile_ = candidate;
    owner_ = executing;
    started_ns_ = now_ns;
    stopping_ = true;
    return Result::accepted;
  }

  bool sample(const GoalId & executing, bool active, std::int64_t now_ns, State & output) noexcept
  {
    if (!active || !stopping_ || owner_ != executing || now_ns < started_ns_) {
      return false;
    }
    output = profile_.sample(static_cast<double>(now_ns - started_ns_) / 1e9);
    return true;
  }

  void clear() noexcept { stopping_ = false; }
  const Profile & profile() const noexcept { return profile_; }
  std::int64_t started_ns() const noexcept { return started_ns_; }

private:
  Profile profile_;
  GoalId owner_{};
  std::int64_t started_ns_{};
  bool stopping_{};
};
}  // namespace execution_stop
