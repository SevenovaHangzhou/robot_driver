#include "dm_swerve_driver/external_steering_encoder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dm_swerve_driver {
namespace {

constexpr double kTau{2.0 * kPi};

[[nodiscard]] std::uint64_t total_positions(const EncoderHardwareInfo & hardware) noexcept
{
  return static_cast<std::uint64_t>(hardware.counts_per_revolution) *
         static_cast<std::uint64_t>(hardware.distinguishable_revolutions);
}

[[nodiscard]] bool position_valid(
  std::uint32_t position, const EncoderHardwareInfo & hardware) noexcept
{
  return hardware.counts_per_revolution > 0U &&
         hardware.distinguishable_revolutions > 0U &&
         static_cast<std::uint64_t>(position) < total_positions(hardware);
}

[[nodiscard]] double axis_motion_between_positions(
  std::uint32_t first,
  std::uint32_t second,
  const ExternalSteeringEncoderConfig & config) noexcept
{
  const auto count_delta = static_cast<std::int64_t>(first) -
    static_cast<std::int64_t>(second);
  const double encoder_turns{
    static_cast<double>(count_delta) /
    static_cast<double>(config.expected_counts_per_revolution)};
  return std::abs(
    encoder_turns * kTau * static_cast<double>(config.pinion_teeth) /
    static_cast<double>(config.ring_gear_teeth));
}

void validate_sample(const SteeringAngleSample & sample)
{
  if (sample.fresh && !std::isfinite(sample.angle_rad)) {
    throw std::invalid_argument{"fresh steering angle samples must be finite"};
  }
}

}  // namespace

bool operator==(
  const EncoderHardwareInfo & lhs, const EncoderHardwareInfo & rhs) noexcept
{
  return lhs.counts_per_revolution == rhs.counts_per_revolution &&
         lhs.distinguishable_revolutions == rhs.distinguishable_revolutions;
}

bool operator==(
  const EncoderPersistentRecord & lhs, const EncoderPersistentRecord & rhs) noexcept
{
  return lhs.position == rhs.position && lhs.hardware == rhs.hardware;
}

void validate_external_encoder_config(const ExternalSteeringEncoderConfig & config)
{
  const bool integer_fields_valid = config.expected_counts_per_revolution > 0U &&
    config.expected_distinguishable_revolutions > 0U &&
    config.ring_gear_teeth > 0U && config.pinion_teeth > 0U &&
    (config.direction == -1 || config.direction == 1);
  const bool scalar_fields_valid = std::isfinite(config.installation_offset_rad) &&
    std::isfinite(config.source_disagreement_threshold_rad) &&
    config.source_disagreement_threshold_rad >= 0.0 &&
    std::isfinite(config.maximum_offline_axis_motion_rad) &&
    config.maximum_offline_axis_motion_rad >= 0.0;
  if (!integer_fields_valid || !scalar_fields_valid) {
    throw std::invalid_argument{"invalid external steering encoder configuration"};
  }
}

double external_encoder_position_to_axis_angle(
  std::uint32_t position, const ExternalSteeringEncoderConfig & config)
{
  validate_external_encoder_config(config);
  const double encoder_turns{
    static_cast<double>(position) /
    static_cast<double>(config.expected_counts_per_revolution)};
  const double direction{static_cast<double>(config.direction)};
  return direction * encoder_turns * kTau * static_cast<double>(config.pinion_teeth) /
         static_cast<double>(config.ring_gear_teeth) - config.installation_offset_rad;
}

bool EncoderStartupResult::accepted() const noexcept
{
  return failure == EncoderStartupFailure::none;
}

EncoderStartupResult validate_external_encoder_startup(
  const ExternalSteeringEncoderConfig & config,
  const EncoderHardwareInfo & hardware,
  std::uint32_t current_position,
  const std::optional<EncoderPersistentRecord> & persisted,
  double motor_axis_angle_rad,
  const SteeringAngleLimits & limits)
{
  validate_external_encoder_config(config);
  if (!std::isfinite(motor_axis_angle_rad) || !valid_steering_angle_limits(limits)) {
    throw std::invalid_argument{"invalid steering startup evidence"};
  }
  const EncoderHardwareInfo expected{
    config.expected_counts_per_revolution,
    config.expected_distinguishable_revolutions};
  if (!(hardware == expected)) {
    return EncoderStartupResult{EncoderStartupFailure::hardware_mismatch};
  }
  if (!persisted.has_value()) {
    return EncoderStartupResult{EncoderStartupFailure::snapshot_missing};
  }
  if (!(persisted->hardware == hardware)) {
    return EncoderStartupResult{EncoderStartupFailure::snapshot_hardware_mismatch};
  }
  if (!position_valid(current_position, hardware) ||
    !position_valid(persisted->position, hardware))
  {
    return EncoderStartupResult{EncoderStartupFailure::position_out_of_range};
  }
  if (axis_motion_between_positions(current_position, persisted->position, config) >
    config.maximum_offline_axis_motion_rad)
  {
    return EncoderStartupResult{EncoderStartupFailure::persistent_position_jump};
  }

  const double axis_angle{external_encoder_position_to_axis_angle(current_position, config)};
  if (!steering_measurement_within_tolerance(axis_angle, limits)) {
    return EncoderStartupResult{EncoderStartupFailure::mechanical_limit, axis_angle};
  }
  const double source_difference{std::abs(axis_angle - motor_axis_angle_rad)};
  if (source_difference > config.source_disagreement_threshold_rad) {
    return EncoderStartupResult{
      EncoderStartupFailure::source_disagreement, axis_angle, source_difference};
  }
  return EncoderStartupResult{EncoderStartupFailure::none, axis_angle, source_difference};
}

SteeringAngleSourceSelector::SteeringAngleSourceSelector(
  double disagreement_threshold_rad,
  double maximum_rejoin_correction_rad)
: disagreement_threshold_rad_{disagreement_threshold_rad},
  maximum_rejoin_correction_rad_{maximum_rejoin_correction_rad}
{
  if (!std::isfinite(disagreement_threshold_rad_) || disagreement_threshold_rad_ < 0.0 ||
    !std::isfinite(maximum_rejoin_correction_rad_) || maximum_rejoin_correction_rad_ <= 0.0)
  {
    throw std::invalid_argument{"steering source thresholds must be finite and valid"};
  }
}

SteeringAngleSelection SteeringAngleSourceSelector::select(
  const SteeringAngleSample & external,
  const SteeringAngleSample & motor)
{
  validate_sample(external);
  validate_sample(motor);
  if (external.fresh && motor.fresh &&
    std::abs(external.angle_rad - motor.angle_rad) > disagreement_threshold_rad_)
  {
    return unavailable(true);
  }

  const SteeringAngleSource previous_source{source_};
  if (external.fresh) {
    const bool recovering = source_ != SteeringAngleSource::external_encoder &&
      selected_angle_valid_;
    if (recovering) {
      external_alignment_rad_ = selected_angle_rad_ - external.angle_rad;
    } else if (source_ == SteeringAngleSource::external_encoder) {
      external_alignment_rad_ -= std::clamp(
        external_alignment_rad_,
        -maximum_rejoin_correction_rad_,
        maximum_rejoin_correction_rad_);
    }
    selected_angle_rad_ = external.angle_rad + external_alignment_rad_;
    selected_angle_valid_ = true;
    source_ = SteeringAngleSource::external_encoder;
    if (motor.fresh) {
      motor_alignment_rad_ = selected_angle_rad_ - motor.angle_rad;
      motor_alignment_valid_ = true;
    }
    return SteeringAngleSelection{
      selected_angle_rad_, source_, true, false, source_ != previous_source, false};
  }

  if (motor.fresh && motor_alignment_valid_) {
    selected_angle_rad_ = motor.angle_rad + motor_alignment_rad_;
    selected_angle_valid_ = true;
    source_ = SteeringAngleSource::motor_backup;
    return SteeringAngleSelection{
      selected_angle_rad_, source_, true, true, source_ != previous_source, false};
  }
  return unavailable(false);
}

void SteeringAngleSourceSelector::reset() noexcept
{
  source_ = SteeringAngleSource::unavailable;
  selected_angle_valid_ = false;
  selected_angle_rad_ = 0.0;
  motor_alignment_valid_ = false;
  motor_alignment_rad_ = 0.0;
  external_alignment_rad_ = 0.0;
}

SteeringAngleSelection SteeringAngleSourceSelector::unavailable(bool disagreement) noexcept
{
  const bool changed{source_ != SteeringAngleSource::unavailable};
  source_ = SteeringAngleSource::unavailable;
  return SteeringAngleSelection{
    selected_angle_rad_, source_, false, true, changed, disagreement};
}

}  // namespace dm_swerve_driver
