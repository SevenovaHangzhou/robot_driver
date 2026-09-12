#include "dm_swerve_driver/ros_params.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace dm_swerve_driver {
namespace {

template<typename T>
void declare_if_missing(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  const T & default_value)
{
  if (!node.has_parameter(name)) {
    static_cast<void>(node.declare_parameter<T>(name, default_value));
  }
}

template<typename T, std::size_t Size>
[[nodiscard]] std::vector<T> to_vector(const std::array<T, Size> & values)
{
  return std::vector<T>(values.begin(), values.end());
}

template<typename Integer, std::size_t Size>
[[nodiscard]] std::vector<std::int64_t> to_integer_vector(
  const std::array<Integer, Size> & values)
{
  std::vector<std::int64_t> result;
  result.reserve(Size);
  std::transform(values.begin(), values.end(), std::back_inserter(result), [](Integer value) {
      return static_cast<std::int64_t>(value);
    });
  return result;
}

template<typename Integer, std::size_t Size>
[[nodiscard]] std::array<Integer, Size> integer_array(
  const std::vector<std::int64_t> & values,
  const char * name)
{
  if (values.size() != Size) {
    throw std::invalid_argument{
      std::string{name} + " must contain exactly " + std::to_string(Size) + " values"};
  }
  std::array<Integer, Size> result{};
  for (std::size_t index{0U}; index < Size; ++index) {
    bool invalid{false};
    if constexpr (std::is_signed_v<Integer>) {
      invalid = values[index] < static_cast<std::int64_t>(
        std::numeric_limits<Integer>::lowest()) ||
        values[index] > static_cast<std::int64_t>(std::numeric_limits<Integer>::max());
    } else {
      invalid = values[index] < 0 || static_cast<std::uint64_t>(values[index]) >
        static_cast<std::uint64_t>(std::numeric_limits<Integer>::max());
    }
    if (invalid) {
      throw std::invalid_argument{std::string{name} + " contains an out-of-range value"};
    }
    result[index] = static_cast<Integer>(values[index]);
  }
  return result;
}

template<std::size_t Size>
[[nodiscard]] std::array<double, Size> double_array(
  const std::vector<double> & values,
  const char * name)
{
  if (values.size() != Size) {
    throw std::invalid_argument{
      std::string{name} + " must contain exactly " + std::to_string(Size) + " values"};
  }
  std::array<double, Size> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

}  // namespace

void declare_kinco_parameters(rclcpp_lifecycle::LifecycleNode & node)
{
  const KincoParameters defaults{};
  declare_if_missing(node, "kinco.ethercat.slave_positions", to_integer_vector(defaults.slave_positions));
  declare_if_missing(node, "kinco.ethercat.vendor_id", static_cast<std::int64_t>(defaults.vendor_id));
  declare_if_missing(node, "kinco.ethercat.product_code", static_cast<std::int64_t>(defaults.product_code));
  declare_if_missing(node, "kinco.ethercat.master_index",
    static_cast<std::int64_t>(defaults.ethercat_master_index));
  declare_if_missing(node, "kinco.ethercat.dc_cycle_ns", defaults.dc_cycle_ns);
  declare_if_missing(node, "kinco.ethercat.pdo_watchdog_ms", defaults.pdo_watchdog_ms);
  declare_if_missing(node, "kinco.ethercat.startup_cycle_limit",
    static_cast<std::int64_t>(defaults.startup_cycle_limit));
  declare_if_missing(node, "kinco.steering_encoder_resolution",
    to_integer_vector(defaults.steering_encoder_resolution));
  declare_if_missing(node, "kinco.drive_encoder_resolution",
    to_integer_vector(defaults.drive_encoder_resolution));
  declare_if_missing(node, "kinco.write_position_feedforward",
    defaults.write_position_feedforward);
  declare_if_missing(node, "kinco.position_velocity_feedforward_raw",
    static_cast<std::int64_t>(defaults.position_velocity_feedforward_raw));
  declare_if_missing(node, "kinco.position_acceleration_feedforward",
    static_cast<std::int64_t>(defaults.position_acceleration_feedforward));
  declare_if_missing(node, "kinco.write_fault_reaction_option",
    defaults.write_fault_reaction_option);
  declare_if_missing(node, "kinco.fault_reaction_option_code",
    static_cast<std::int64_t>(defaults.fault_reaction_option_code));

  declare_if_missing(node, "kinco.encoder.can_interface", defaults.encoder_can_interface);
  declare_if_missing(node, "kinco.encoder.feedback_deadline_us",
    defaults.encoder_feedback_deadline_us);
  declare_if_missing(node, "kinco.encoder.node_ids",
    to_integer_vector(defaults.encoder_node_ids));
  declare_if_missing(node, "kinco.encoder.expected_counts_per_revolution",
    to_integer_vector(defaults.encoder_expected_counts_per_revolution));
  declare_if_missing(node, "kinco.encoder.expected_distinguishable_revolutions",
    to_integer_vector(defaults.encoder_expected_distinguishable_revolutions));
  declare_if_missing(node, "kinco.encoder.ring_gear_teeth",
    to_integer_vector(defaults.encoder_ring_gear_teeth));
  declare_if_missing(node, "kinco.encoder.pinion_teeth",
    to_integer_vector(defaults.encoder_pinion_teeth));
  declare_if_missing(node, "kinco.encoder.direction",
    to_integer_vector(defaults.encoder_direction));
  declare_if_missing(node, "kinco.encoder.installation_offset_rad",
    to_vector(defaults.encoder_installation_offset_rad));
  declare_if_missing(node, "kinco.encoder.source_disagreement_threshold_rad",
    defaults.encoder_source_disagreement_threshold_rad);
  declare_if_missing(node, "kinco.encoder.maximum_offline_axis_motion_rad",
    defaults.encoder_maximum_offline_axis_motion_rad);
  declare_if_missing(node, "kinco.encoder.maximum_rejoin_correction_rad",
    defaults.encoder_maximum_rejoin_correction_rad);
  declare_if_missing(node, "kinco.encoder.snapshot_path", defaults.encoder_snapshot_path);
}

KincoParameters load_kinco_parameters(const rclcpp_lifecycle::LifecycleNode & node)
{
  KincoParameters result;
  result.slave_positions = integer_array<std::uint16_t, kKincoAxisCount>(
    node.get_parameter("kinco.ethercat.slave_positions").as_integer_array(),
    "kinco.ethercat.slave_positions");
  const auto vendor = node.get_parameter("kinco.ethercat.vendor_id").as_int();
  const auto product = node.get_parameter("kinco.ethercat.product_code").as_int();
  if (vendor <= 0 || vendor > std::numeric_limits<std::uint32_t>::max() ||
    product <= 0 || product > std::numeric_limits<std::uint32_t>::max())
  {throw std::invalid_argument{"EtherCAT vendor/product identifiers must be positive uint32"};}
  result.vendor_id = static_cast<std::uint32_t>(vendor);
  result.product_code = static_cast<std::uint32_t>(product);
  const auto master_index = node.get_parameter("kinco.ethercat.master_index").as_int();
  if (master_index < 0 ||
    master_index > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max()))
  {
    throw std::invalid_argument{"kinco.ethercat.master_index is outside uint32 range"};
  }
  result.ethercat_master_index = static_cast<std::uint32_t>(master_index);
  result.dc_cycle_ns = node.get_parameter("kinco.ethercat.dc_cycle_ns").as_int();
  result.pdo_watchdog_ms = node.get_parameter("kinco.ethercat.pdo_watchdog_ms").as_int();
  const auto startup_limit = node.get_parameter("kinco.ethercat.startup_cycle_limit").as_int();
  if (startup_limit < 0) {
    throw std::invalid_argument{"kinco.ethercat.startup_cycle_limit cannot be negative"};
  }
  result.startup_cycle_limit = static_cast<std::size_t>(startup_limit);
  result.steering_encoder_resolution = integer_array<std::uint32_t, kSwerveModuleCount>(
    node.get_parameter("kinco.steering_encoder_resolution").as_integer_array(),
    "kinco.steering_encoder_resolution");
  result.drive_encoder_resolution = integer_array<std::uint32_t, kSwerveModuleCount>(
    node.get_parameter("kinco.drive_encoder_resolution").as_integer_array(),
    "kinco.drive_encoder_resolution");
  result.write_position_feedforward =
    node.get_parameter("kinco.write_position_feedforward").as_bool();
  const auto velocity_ff =
    node.get_parameter("kinco.position_velocity_feedforward_raw").as_int();
  const auto acceleration_ff =
    node.get_parameter("kinco.position_acceleration_feedforward").as_int();
  if (velocity_ff < 0 || velocity_ff > std::numeric_limits<std::uint16_t>::max() ||
    acceleration_ff < 0 || acceleration_ff > std::numeric_limits<std::uint16_t>::max())
  {
    throw std::invalid_argument{"Kinco position feedforward values exceed uint16 range"};
  }
  result.position_velocity_feedforward_raw = static_cast<std::uint16_t>(velocity_ff);
  result.position_acceleration_feedforward = static_cast<std::uint16_t>(acceleration_ff);
  result.write_fault_reaction_option =
    node.get_parameter("kinco.write_fault_reaction_option").as_bool();
  const auto fault_reaction = node.get_parameter("kinco.fault_reaction_option_code").as_int();
  if (fault_reaction < std::numeric_limits<std::int16_t>::lowest() ||
    fault_reaction > std::numeric_limits<std::int16_t>::max())
  {
    throw std::invalid_argument{"Kinco fault reaction option exceeds int16 range"};
  }
  result.fault_reaction_option_code = static_cast<std::int16_t>(fault_reaction);

  result.encoder_can_interface = node.get_parameter("kinco.encoder.can_interface").as_string();
  result.encoder_feedback_deadline_us =
    node.get_parameter("kinco.encoder.feedback_deadline_us").as_int();
  result.encoder_node_ids = integer_array<std::uint16_t, kSwerveModuleCount>(
    node.get_parameter("kinco.encoder.node_ids").as_integer_array(),
    "kinco.encoder.node_ids");
  result.encoder_expected_counts_per_revolution =
    integer_array<std::uint32_t, kSwerveModuleCount>(
    node.get_parameter("kinco.encoder.expected_counts_per_revolution").as_integer_array(),
    "kinco.encoder.expected_counts_per_revolution");
  result.encoder_expected_distinguishable_revolutions =
    integer_array<std::uint32_t, kSwerveModuleCount>(
    node.get_parameter("kinco.encoder.expected_distinguishable_revolutions").as_integer_array(),
    "kinco.encoder.expected_distinguishable_revolutions");
  result.encoder_ring_gear_teeth = integer_array<std::uint32_t, kSwerveModuleCount>(
    node.get_parameter("kinco.encoder.ring_gear_teeth").as_integer_array(),
    "kinco.encoder.ring_gear_teeth");
  result.encoder_pinion_teeth = integer_array<std::uint32_t, kSwerveModuleCount>(
    node.get_parameter("kinco.encoder.pinion_teeth").as_integer_array(),
    "kinco.encoder.pinion_teeth");
  result.encoder_direction = integer_array<int, kSwerveModuleCount>(
    node.get_parameter("kinco.encoder.direction").as_integer_array(),
    "kinco.encoder.direction");
  result.encoder_installation_offset_rad = double_array<kSwerveModuleCount>(
    node.get_parameter("kinco.encoder.installation_offset_rad").as_double_array(),
    "kinco.encoder.installation_offset_rad");
  result.encoder_source_disagreement_threshold_rad =
    node.get_parameter("kinco.encoder.source_disagreement_threshold_rad").as_double();
  result.encoder_maximum_offline_axis_motion_rad =
    node.get_parameter("kinco.encoder.maximum_offline_axis_motion_rad").as_double();
  result.encoder_maximum_rejoin_correction_rad =
    node.get_parameter("kinco.encoder.maximum_rejoin_correction_rad").as_double();
  result.encoder_snapshot_path = node.get_parameter("kinco.encoder.snapshot_path").as_string();
  validate_kinco_parameters(result);
  return result;
}

}  // namespace dm_swerve_driver
