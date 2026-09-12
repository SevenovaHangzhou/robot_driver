#ifndef DM_SWERVE_DRIVER__EXTERNAL_STEERING_ENCODER_HPP_
#define DM_SWERVE_DRIVER__EXTERNAL_STEERING_ENCODER_HPP_

#include <cstdint>
#include <optional>

#include "dm_swerve_driver/swerve_kinematics.hpp"

namespace dm_swerve_driver {

struct EncoderHardwareInfo {
  std::uint32_t counts_per_revolution{0U};
  std::uint32_t distinguishable_revolutions{0U};
};

[[nodiscard]] bool operator==(
  const EncoderHardwareInfo & lhs, const EncoderHardwareInfo & rhs) noexcept;

struct EncoderPersistentRecord {
  std::uint32_t position{0U};
  EncoderHardwareInfo hardware{};
};

[[nodiscard]] bool operator==(
  const EncoderPersistentRecord & lhs, const EncoderPersistentRecord & rhs) noexcept;

struct ExternalSteeringEncoderConfig {
  std::uint32_t expected_counts_per_revolution{0U};
  std::uint32_t expected_distinguishable_revolutions{0U};
  std::uint32_t ring_gear_teeth{0U};
  std::uint32_t pinion_teeth{0U};
  int direction{1};
  double installation_offset_rad{0.0};
  double source_disagreement_threshold_rad{0.0};
  double maximum_offline_axis_motion_rad{0.0};
};

void validate_external_encoder_config(const ExternalSteeringEncoderConfig & config);
[[nodiscard]] double external_encoder_position_to_axis_angle(
  std::uint32_t position, const ExternalSteeringEncoderConfig & config);

enum class EncoderStartupFailure {
  none,
  hardware_mismatch,
  snapshot_missing,
  snapshot_hardware_mismatch,
  position_out_of_range,
  persistent_position_jump,
  mechanical_limit,
  source_disagreement,
};

struct EncoderStartupResult {
  EncoderStartupFailure failure{EncoderStartupFailure::none};
  double axis_angle_rad{0.0};
  double source_difference_rad{0.0};

  [[nodiscard]] bool accepted() const noexcept;
};

[[nodiscard]] EncoderStartupResult validate_external_encoder_startup(
  const ExternalSteeringEncoderConfig & config,
  const EncoderHardwareInfo & hardware,
  std::uint32_t current_position,
  const std::optional<EncoderPersistentRecord> & persisted,
  double motor_axis_angle_rad,
  const SteeringAngleLimits & limits);

struct SteeringAngleSample {
  double angle_rad{0.0};
  bool fresh{false};
};

enum class SteeringAngleSource {
  unavailable,
  external_encoder,
  motor_backup,
};

struct SteeringAngleSelection {
  double angle_rad{0.0};
  SteeringAngleSource source{SteeringAngleSource::unavailable};
  bool valid{false};
  bool degraded{false};
  bool source_changed{false};
  bool disagreement{false};
};

class SteeringAngleSourceSelector final {
public:
  SteeringAngleSourceSelector(
    double disagreement_threshold_rad,
    double maximum_rejoin_correction_rad);

  [[nodiscard]] SteeringAngleSelection select(
    const SteeringAngleSample & external,
    const SteeringAngleSample & motor);
  void reset() noexcept;

private:
  [[nodiscard]] SteeringAngleSelection unavailable(bool disagreement) noexcept;

  double disagreement_threshold_rad_{0.0};
  double maximum_rejoin_correction_rad_{0.0};
  SteeringAngleSource source_{SteeringAngleSource::unavailable};
  bool selected_angle_valid_{false};
  double selected_angle_rad_{0.0};
  bool motor_alignment_valid_{false};
  double motor_alignment_rad_{0.0};
  double external_alignment_rad_{0.0};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__EXTERNAL_STEERING_ENCODER_HPP_
