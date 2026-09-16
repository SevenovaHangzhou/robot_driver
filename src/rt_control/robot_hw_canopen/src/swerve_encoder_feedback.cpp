#include "robot_hw_canopen/swerve_encoder_feedback.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace robot_hw_canopen
{
namespace
{

constexpr double kTwoPi{6.28318530717958647692};
constexpr double kNanosecondsPerMillisecond{1000000.0};
constexpr unsigned int kSnapshotAttempts{3U};

[[nodiscard]] SwerveEncoderSample invalid_sample() noexcept
{
  return {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    false};
}

void validate_axis(
  const SwerveEncoderAxisConfig & axis,
  const std::array<SwerveEncoderAxisConfig, kSwerveEncoderCount> & config,
  std::size_t index)
{
  if (axis.node_id == 0U || axis.node_id > 127U) {
    throw std::invalid_argument{"swerve encoder node ID must be in [1, 127]"};
  }
  if (axis.counts_per_revolution == 0U || axis.ring_gear_teeth == 0U ||
    axis.pinion_gear_teeth == 0U)
  {
    throw std::invalid_argument{"swerve encoder counts and gear teeth must be positive"};
  }
  if (axis.direction != -1 && axis.direction != 1) {
    throw std::invalid_argument{"swerve encoder direction must be -1 or 1"};
  }
  if (!std::isfinite(axis.installation_offset_rad)) {
    throw std::invalid_argument{"swerve encoder installation offset must be finite"};
  }
  for (std::size_t previous{0U}; previous < index; ++previous) {
    if (config[previous].node_id == axis.node_id) {
      throw std::invalid_argument{"swerve encoder node IDs must be unique"};
    }
  }
  const double radians_per_count{
    kTwoPi * static_cast<double>(axis.pinion_gear_teeth) /
    (static_cast<double>(axis.counts_per_revolution) *
    static_cast<double>(axis.ring_gear_teeth))};
  if (!std::isfinite(radians_per_count) || radians_per_count <= 0.0) {
    throw std::invalid_argument{"swerve encoder scale is not representable"};
  }
}

}  // namespace

static_assert(
  std::atomic<std::uint64_t>::is_always_lock_free &&
  std::atomic<std::int64_t>::is_always_lock_free &&
  std::atomic<std::uint32_t>::is_always_lock_free,
  "swerve encoder snapshots must remain lock-free");

SwerveEncoderFeedback::SwerveEncoderFeedback(
  std::array<SwerveEncoderAxisConfig, kSwerveEncoderCount> config)
: config_{std::move(config)}
{
  for (std::size_t index{0U}; index < config_.size(); ++index) {
    validate_axis(config_[index], config_, index);
  }
}

bool SwerveEncoderFeedback::observe(
  std::uint8_t node_id, std::uint16_t index, std::uint8_t subindex,
  std::uint32_t raw_position, std::int64_t received_steady_ns) noexcept
{
  if (index != kSwerveEncoderPositionIndex ||
    subindex != kSwerveEncoderPositionSubindex || received_steady_ns <= 0)
  {
    return false;
  }
  const std::size_t axis{axis_for_node(node_id)};
  if (axis == kSwerveEncoderCount) {
    return false;
  }

  auto & slot = slots_[axis];
  std::uint64_t sequence{slot.sequence.load(std::memory_order_acquire)};
  if ((sequence & 1U) != 0U ||
    !slot.sequence.compare_exchange_strong(
      sequence, sequence + 1U, std::memory_order_acq_rel, std::memory_order_acquire))
  {
    return false;
  }
  slot.raw_position.store(raw_position, std::memory_order_relaxed);
  slot.received_steady_ns.store(received_steady_ns, std::memory_order_relaxed);
  slot.sequence.store(sequence + 2U, std::memory_order_release);
  return true;
}

std::array<SwerveEncoderSample, kSwerveEncoderCount>
SwerveEncoderFeedback::sample(std::int64_t now_steady_ns) const noexcept
{
  std::array<SwerveEncoderSample, kSwerveEncoderCount> result{};
  for (std::size_t axis{0U}; axis < result.size(); ++axis) {
    result[axis] = sample_axis(axis, now_steady_ns);
  }
  return result;
}

std::size_t SwerveEncoderFeedback::axis_for_node(std::uint8_t node_id) const noexcept
{
  for (std::size_t axis{0U}; axis < config_.size(); ++axis) {
    if (config_[axis].node_id == node_id) {
      return axis;
    }
  }
  return kSwerveEncoderCount;
}

SwerveEncoderSample SwerveEncoderFeedback::sample_axis(
  std::size_t axis, std::int64_t now_steady_ns) const noexcept
{
  std::uint32_t raw_position{0U};
  std::int64_t received_steady_ns{0};
  bool coherent{false};
  const auto & slot = slots_[axis];
  for (unsigned int attempt{0U}; attempt < kSnapshotAttempts; ++attempt) {
    const std::uint64_t before{slot.sequence.load(std::memory_order_acquire)};
    if ((before & 1U) != 0U) {
      continue;
    }
    raw_position = slot.raw_position.load(std::memory_order_relaxed);
    received_steady_ns = slot.received_steady_ns.load(std::memory_order_relaxed);
    const std::uint64_t after{slot.sequence.load(std::memory_order_acquire)};
    if (before == after) {
      coherent = true;
      break;
    }
  }
  if (!coherent || received_steady_ns <= 0 || now_steady_ns < received_steady_ns) {
    return invalid_sample();
  }

  const auto & config = config_[axis];
  const double radians_per_count{
    kTwoPi * static_cast<double>(config.pinion_gear_teeth) /
    (static_cast<double>(config.counts_per_revolution) *
    static_cast<double>(config.ring_gear_teeth))};
  const double position{
    static_cast<double>(config.direction) *
    static_cast<double>(raw_position) * radians_per_count -
    config.installation_offset_rad};
  const double age{
    static_cast<double>(now_steady_ns - received_steady_ns) /
    kNanosecondsPerMillisecond};
  if (!std::isfinite(position) || !std::isfinite(age)) {
    return invalid_sample();
  }
  return {position, age, true};
}

}  // namespace robot_hw_canopen
