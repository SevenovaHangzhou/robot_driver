#ifndef DM_SWERVE_DRIVER__KINCO_PARAMS_HPP_
#define DM_SWERVE_DRIVER__KINCO_PARAMS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "dm_swerve_driver/canopen_encoder.hpp"
#include "dm_swerve_driver/kinco_backend.hpp"
#include "dm_swerve_driver/params.hpp"

namespace dm_swerve_driver {

struct KincoEthercatRuntimeConfig {
  std::uint32_t master_index{0U};
  std::int64_t dc_cycle_ns{0};
  std::int64_t pdo_watchdog_ms{0};
};

struct KincoParameters {
  std::array<std::uint16_t, kKincoAxisCount> slave_positions{0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U};
  std::uint32_t vendor_id{0x00681168U};
  std::uint32_t product_code{0x00464445U};
  std::uint32_t ethercat_master_index{0U};
  std::int64_t dc_cycle_ns{0};
  std::int64_t pdo_watchdog_ms{0};
  std::size_t startup_cycle_limit{5000U};
  std::array<std::uint32_t, kSwerveModuleCount> steering_encoder_resolution{};
  std::array<std::uint32_t, kSwerveModuleCount> drive_encoder_resolution{};
  bool write_position_feedforward{false};
  std::uint16_t position_velocity_feedforward_raw{256U};
  std::uint16_t position_acceleration_feedforward{32767U};
  bool write_fault_reaction_option{false};
  std::int16_t fault_reaction_option_code{0};

  std::string encoder_can_interface{"can0"};
  std::int64_t encoder_feedback_deadline_us{2000};
  std::array<std::uint16_t, kSwerveModuleCount> encoder_node_ids{1U, 2U, 3U, 4U};
  std::array<std::uint32_t, kSwerveModuleCount>
  encoder_expected_counts_per_revolution{};
  std::array<std::uint32_t, kSwerveModuleCount>
  encoder_expected_distinguishable_revolutions{};
  std::array<std::uint32_t, kSwerveModuleCount> encoder_ring_gear_teeth{};
  std::array<std::uint32_t, kSwerveModuleCount> encoder_pinion_teeth{};
  std::array<int, kSwerveModuleCount> encoder_direction{1, 1, 1, 1};
  std::array<double, kSwerveModuleCount> encoder_installation_offset_rad{};
  double encoder_source_disagreement_threshold_rad{0.0};
  double encoder_maximum_offline_axis_motion_rad{0.0};
  double encoder_maximum_rejoin_correction_rad{0.0};
  std::string encoder_snapshot_path{};
};

[[nodiscard]] std::vector<std::string> kinco_parameter_errors(
  const KincoParameters & parameters);
void validate_kinco_parameters(const KincoParameters & parameters);

[[nodiscard]] KincoEthercatRuntimeConfig kinco_ethercat_runtime_config(
  const KincoParameters & parameters);
[[nodiscard]] KincoSwerveHardwareConfig kinco_hardware_config(
  const DriverParameters & common,
  const KincoParameters & kinco);
[[nodiscard]] ExternalSteeringEncoderConfig external_encoder_config(
  const KincoParameters & parameters,
  std::size_t module_index);
[[nodiscard]] EncoderCanopenConfig encoder_canopen_config(
  const KincoParameters & parameters);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__KINCO_PARAMS_HPP_
