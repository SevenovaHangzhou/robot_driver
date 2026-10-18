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
  std::atomic<std::uint8_t>::is_always_lock_free && std::atomic<std::uint32_t>::is_always_lock_free,
  "swerve encoder triple buffers must remain lock-free");

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

  // Single writer: fill the back buffer, then publish it as the fresh middle buffer.
  auto & slot = slots_[axis];
  slot.frames[slot.back] = {
    raw_position, received_steady_ns, slot.nmt_epoch.load(std::memory_order_acquire)};
  slot.back = static_cast<std::uint8_t>(
    slot.middle.exchange(
      static_cast<std::uint8_t>(slot.back | kFreshBit), std::memory_order_acq_rel) &
    kIndexMask);
  return true;
}

bool SwerveEncoderFeedback::observe_nmt(std::uint8_t node_id, bool operational) noexcept
{
  const std::size_t axis{axis_for_node(node_id)};
  if (axis == kSwerveEncoderCount) {
    return false;
  }
  if (!operational) {
    slots_[axis].nmt_epoch.fetch_add(1U, std::memory_order_acq_rel);
  }
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
  // Single reader: take the newest published buffer if there is one; otherwise keep the
  // current front buffer. Staleness is judged by age only.
  const auto & slot = slots_[axis];
  if ((slot.middle.load(std::memory_order_acquire) & kFreshBit) != 0U) {
    slot.front = static_cast<std::uint8_t>(
      slot.middle.exchange(slot.front, std::memory_order_acq_rel) & kIndexMask);
    slot.has_frame = true;
  }
  if (!slot.has_frame) {
    return invalid_sample();
  }
  const Frame frame = slot.frames[slot.front];
  const std::uint32_t raw_position{frame.raw_position};
  if (
    frame.received_steady_ns <= 0 ||
    frame.nmt_epoch != slot.nmt_epoch.load(std::memory_order_acquire)) {
    return invalid_sample();
  }
  // A frame stored after the reader took ``now`` is fresh (age 0), not invalid.
  const std::int64_t received_steady_ns{frame.received_steady_ns};
  if (now_steady_ns < received_steady_ns) {
    now_steady_ns = received_steady_ns;
  }

  const auto & config = config_[axis];
  const double radians_per_count{
    kTwoPi * static_cast<double>(config.pinion_gear_teeth) /
    (static_cast<double>(config.counts_per_revolution) *
    static_cast<double>(config.ring_gear_teeth))};
  double position{
    static_cast<double>(config.direction) * static_cast<double>(raw_position) * radians_per_count -
    config.installation_offset_rad};
  if (config.encoder_revolutions > 0U) {
    const auto range =
      static_cast<std::uint64_t>(config.counts_per_revolution) * config.encoder_revolutions;
    if (raw_position >= range) {
      return invalid_sample();
    }
    position = std::remainder(position, static_cast<double>(range) * radians_per_count);
  }
  const double age{
    static_cast<double>(now_steady_ns - received_steady_ns) /
    kNanosecondsPerMillisecond};
  if (!std::isfinite(position) || !std::isfinite(age)) {
    return invalid_sample();
  }
  return {position, age, true};
}

}  // namespace robot_hw_canopen
