#include "dm_swerve_driver/kinco_params.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dm_swerve_driver {
namespace {

template<typename T, std::size_t Size>
[[nodiscard]] bool all_positive(const std::array<T, Size> & values) noexcept
{
  return std::all_of(values.begin(), values.end(), [](T value) {return value > 0;});
}

[[nodiscard]] bool all_finite(
  const std::array<double, kSwerveModuleCount> & values) noexcept
{
  return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

void add_if(std::vector<std::string> & errors, bool condition, const char * message)
{
  if (condition) {
    errors.emplace_back(message);
  }
}

[[nodiscard]] std::string join_errors(const std::vector<std::string> & errors)
{
  std::ostringstream stream;
  for (std::size_t index{0U}; index < errors.size(); ++index) {
    if (index != 0U) {
      stream << "; ";
    }
    stream << errors[index];
  }
  return stream.str();
}

void validate_module_index(std::size_t module_index)
{
  if (module_index >= kSwerveModuleCount) {
    throw std::out_of_range{"swerve module index is out of range"};
  }
}

}  // namespace

std::vector<std::string> kinco_parameter_errors(const KincoParameters & parameters)
{
  std::vector<std::string> errors;
  auto slave_positions = parameters.slave_positions;
  std::sort(slave_positions.begin(), slave_positions.end());
  add_if(errors, std::adjacent_find(slave_positions.begin(), slave_positions.end()) !=
    slave_positions.end(), "kinco.ethercat.slave_positions must be unique");
  add_if(errors, parameters.vendor_id == 0U || parameters.product_code == 0U,
    "kinco.ethercat vendor and product identifiers must be positive");
  add_if(errors, parameters.dc_cycle_ns <= 0,
    "kinco.ethercat.dc_cycle_ns must be confirmed and positive");
  add_if(errors, parameters.pdo_watchdog_ms <= 0,
    "kinco.ethercat.pdo_watchdog_ms must be confirmed and positive");
  add_if(errors, parameters.dc_cycle_ns > std::numeric_limits<std::uint32_t>::max(),
    "kinco.ethercat.dc_cycle_ns exceeds uint32 range");
  add_if(errors, parameters.pdo_watchdog_ms > 65535,
    "kinco.ethercat.pdo_watchdog_ms exceeds supported watchdog range");
  if (parameters.dc_cycle_ns > 0 && parameters.pdo_watchdog_ms > 0 &&
    parameters.pdo_watchdog_ms <= 65535)
  {
    const std::int64_t watchdog_ns = parameters.pdo_watchdog_ms * 1000000;
    add_if(errors, watchdog_ns <= parameters.dc_cycle_ns,
      "kinco.ethercat.pdo_watchdog_ms must exceed one DC cycle");
  }
  add_if(errors, parameters.startup_cycle_limit == 0U,
    "kinco.ethercat.startup_cycle_limit must be positive");
  add_if(errors, !all_positive(parameters.steering_encoder_resolution),
    "kinco.steering_encoder_resolution values must be confirmed and positive");
  add_if(errors, !all_positive(parameters.drive_encoder_resolution),
    "kinco.drive_encoder_resolution values must be confirmed and positive");
  add_if(errors, parameters.write_position_feedforward &&
    parameters.position_velocity_feedforward_raw > 256U,
    "kinco.position_velocity_feedforward_raw must be in [0, 256]");
  add_if(errors, parameters.write_position_feedforward &&
    (parameters.position_acceleration_feedforward < 10U ||
    parameters.position_acceleration_feedforward > 32767U),
    "kinco.position_acceleration_feedforward must be in [10, 32767]");
  add_if(errors, parameters.write_fault_reaction_option &&
    (parameters.fault_reaction_option_code < 0 || parameters.fault_reaction_option_code > 2),
    "kinco.fault_reaction_option_code must be 0, 1 or 2");

  add_if(errors, parameters.encoder_can_interface.empty(),
    "kinco.encoder.can_interface must not be empty");
  add_if(errors, parameters.encoder_feedback_deadline_us <= 0,
    "kinco.encoder.feedback_deadline_us must be positive");
  auto sorted_nodes = parameters.encoder_node_ids;
  std::sort(sorted_nodes.begin(), sorted_nodes.end());
  const bool node_range_invalid = std::any_of(
    sorted_nodes.begin(), sorted_nodes.end(), [](std::uint16_t value) {
      return value == 0U || value > 127U;
    });
  add_if(errors, node_range_invalid ||
    std::adjacent_find(sorted_nodes.begin(), sorted_nodes.end()) != sorted_nodes.end(),
    "kinco.encoder_node_ids must be unique and in [1, 127]");
  add_if(errors, !all_positive(parameters.encoder_expected_counts_per_revolution),
    "kinco.encoder_expected_counts_per_revolution values must be confirmed and positive");
  add_if(errors, !all_positive(parameters.encoder_expected_distinguishable_revolutions),
    "kinco.encoder_expected_distinguishable_revolutions values must be confirmed and positive");
  add_if(errors, !all_positive(parameters.encoder_ring_gear_teeth),
    "kinco.encoder_ring_gear_teeth values must be exact positive integers");
  add_if(errors, !all_positive(parameters.encoder_pinion_teeth),
    "kinco.encoder_pinion_teeth values must be exact positive integers");
  add_if(errors, std::any_of(
      parameters.encoder_direction.begin(), parameters.encoder_direction.end(),
      [](int value) {return value != -1 && value != 1;}),
    "kinco.encoder_direction values must be -1 or 1");
  add_if(errors, !all_finite(parameters.encoder_installation_offset_rad),
    "kinco.encoder_installation_offset_rad values must be finite");
  add_if(errors, !std::isfinite(parameters.encoder_source_disagreement_threshold_rad) ||
    parameters.encoder_source_disagreement_threshold_rad <= 0.0,
    "kinco.encoder.source_disagreement_threshold_rad must be finite and positive");
  add_if(errors, !std::isfinite(parameters.encoder_maximum_offline_axis_motion_rad) ||
    parameters.encoder_maximum_offline_axis_motion_rad < 0.0,
    "kinco.encoder.maximum_offline_axis_motion_rad must be finite and nonnegative");
  add_if(errors, !std::isfinite(parameters.encoder_maximum_rejoin_correction_rad) ||
    parameters.encoder_maximum_rejoin_correction_rad <= 0.0,
    "kinco.encoder.maximum_rejoin_correction_rad must be finite and positive");
  add_if(errors, parameters.encoder_snapshot_path.empty(),
    "kinco.encoder_snapshot_path must not be empty");
  return errors;
}

void validate_kinco_parameters(const KincoParameters & parameters)
{
  const auto errors = kinco_parameter_errors(parameters);
  if (!errors.empty()) {
    throw std::invalid_argument{join_errors(errors)};
  }
}

KincoEthercatRuntimeConfig kinco_ethercat_runtime_config(
  const KincoParameters & parameters)
{
  validate_kinco_parameters(parameters);
  return KincoEthercatRuntimeConfig{
    parameters.ethercat_master_index,
    parameters.dc_cycle_ns,
    parameters.pdo_watchdog_ms};
}

KincoSwerveHardwareConfig kinco_hardware_config(
  const DriverParameters & common,
  const KincoParameters & kinco)
{
  validate_kinco_parameters(kinco);
  const SteeringAngleLimits steering_limits{
    common.steering.joint_limit_min_rad,
    common.steering.joint_limit_max_rad,
    common.steering.joint_limit_margin_rad,
    common.steering.joint_limit_tolerance_rad};
  const bool common_geometry_valid = std::isfinite(common.steering.gear_ratio) &&
    common.steering.gear_ratio > 0.0 && std::isfinite(common.drive.gear_ratio) &&
    common.drive.gear_ratio > 0.0 && std::isfinite(common.chassis.wheel_radius_m) &&
    common.chassis.wheel_radius_m > 0.0 &&
    all_finite(common.steering.zero_offset_rad) &&
    valid_steering_angle_limits(steering_limits);
  if (!common_geometry_valid) {
    throw std::invalid_argument{"invalid common geometry for Kinco hardware"};
  }
  return KincoSwerveHardwareConfig{
    kinco.steering_encoder_resolution,
    kinco.drive_encoder_resolution,
    common.steering.gear_ratio,
    common.drive.gear_ratio,
    common.chassis.wheel_radius_m,
    common.steering.zero_offset_rad,
    common.steering.inverted,
    common.drive.inverted,
    steering_limits,
    kinco.startup_cycle_limit,
    kinco.write_position_feedforward ?
    std::optional<std::uint16_t>{kinco.position_velocity_feedforward_raw} : std::nullopt,
    kinco.write_position_feedforward ?
    std::optional<std::uint16_t>{kinco.position_acceleration_feedforward} : std::nullopt,
    kinco.write_fault_reaction_option ?
    std::optional<std::int16_t>{kinco.fault_reaction_option_code} : std::nullopt};
}

ExternalSteeringEncoderConfig external_encoder_config(
  const KincoParameters & parameters,
  std::size_t module_index)
{
  validate_kinco_parameters(parameters);
  validate_module_index(module_index);
  return ExternalSteeringEncoderConfig{
    parameters.encoder_expected_counts_per_revolution[module_index],
    parameters.encoder_expected_distinguishable_revolutions[module_index],
    parameters.encoder_ring_gear_teeth[module_index],
    parameters.encoder_pinion_teeth[module_index],
    parameters.encoder_direction[module_index],
    parameters.encoder_installation_offset_rad[module_index],
    parameters.encoder_source_disagreement_threshold_rad,
    parameters.encoder_maximum_offline_axis_motion_rad};
}

EncoderCanopenConfig encoder_canopen_config(const KincoParameters & parameters)
{
  validate_kinco_parameters(parameters);
  std::array<std::uint8_t, kSwerveModuleCount> nodes{};
  std::transform(
    parameters.encoder_node_ids.begin(), parameters.encoder_node_ids.end(), nodes.begin(),
    [](std::uint16_t value) {return static_cast<std::uint8_t>(value);});
  return EncoderCanopenConfig{nodes, parameters.encoder_feedback_deadline_us};
}

}  // namespace dm_swerve_driver
