#include "rt_diagnostics/diagnostics_topology.hpp"

#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace rt_diagnostics
{

DiagnosticsTopology DiagnosticsTopology::create(
  std::vector<std::string> ethercat_joint_names,
  const std::vector<std::int64_t> & ethercat_ring_positions,
  std::vector<std::string> ethercat_sensor_names,
  const std::vector<std::int64_t> & ethercat_sensor_ring_positions,
  std::int64_t ethercat_expected_responders,
  const std::vector<std::int64_t> & canopen_node_ids)
{
  if (ethercat_joint_names.empty()) {
    throw std::invalid_argument("EtherCAT diagnostic topology must contain at least one axis");
  }
  if (ethercat_joint_names.size() != ethercat_ring_positions.size()) {
    throw std::invalid_argument("EtherCAT joint names and ring positions must have equal length");
  }
  if (ethercat_sensor_names.size() != ethercat_sensor_ring_positions.size()) {
    throw std::invalid_argument(
            "EtherCAT sensor names and ring positions must have equal length");
  }
  if (
    ethercat_expected_responders <= 0 ||
    ethercat_expected_responders > std::numeric_limits<std::uint16_t>::max())
  {
    throw std::invalid_argument("EtherCAT expected responder count is outside uint16 range");
  }
  if (
    static_cast<std::uint64_t>(ethercat_expected_responders) <
    ethercat_joint_names.size() + ethercat_sensor_names.size())
  {
    throw std::invalid_argument(
            "EtherCAT expected responder count is smaller than the configured resource count");
  }

  std::unordered_set<std::string> unique_resource_names;
  std::unordered_set<std::uint16_t> unique_ring_positions;
  std::vector<EthercatAxis> ethercat_axes;
  ethercat_axes.reserve(ethercat_joint_names.size());
  for (std::size_t index = 0; index < ethercat_joint_names.size(); ++index) {
    auto & joint_name = ethercat_joint_names[index];
    if (joint_name.empty() || !unique_resource_names.insert(joint_name).second) {
      throw std::invalid_argument("EtherCAT joint names must be nonempty and unique");
    }
    const auto position = ethercat_ring_positions[index];
    if (position < 0 || position >= ethercat_expected_responders) {
      throw std::invalid_argument("EtherCAT ring position is outside the configured responder ring");
    }
    const auto typed_position = static_cast<std::uint16_t>(position);
    if (!unique_ring_positions.insert(typed_position).second) {
      throw std::invalid_argument("EtherCAT ring positions must be unique");
    }
    ethercat_axes.push_back({std::move(joint_name), typed_position});
  }

  std::vector<EthercatSensor> ethercat_sensors;
  ethercat_sensors.reserve(ethercat_sensor_names.size());
  for (std::size_t index = 0; index < ethercat_sensor_names.size(); ++index) {
    auto & sensor_name = ethercat_sensor_names[index];
    if (sensor_name.empty() || !unique_resource_names.insert(sensor_name).second) {
      throw std::invalid_argument(
              "EtherCAT resource names must be nonempty and unique across axes and sensors");
    }
    const auto position = ethercat_sensor_ring_positions[index];
    if (position < 0 || position >= ethercat_expected_responders) {
      throw std::invalid_argument(
              "EtherCAT sensor ring position is outside the configured responder ring");
    }
    const auto typed_position = static_cast<std::uint16_t>(position);
    if (!unique_ring_positions.insert(typed_position).second) {
      throw std::invalid_argument(
              "EtherCAT ring positions must be unique across axes and sensors");
    }
    ethercat_sensors.push_back({std::move(sensor_name), typed_position});
  }

  if (canopen_node_ids.empty()) {
    throw std::invalid_argument("CANopen diagnostic topology must contain at least one node");
  }
  std::unordered_set<std::uint8_t> unique_node_ids;
  std::vector<std::uint8_t> typed_node_ids;
  typed_node_ids.reserve(canopen_node_ids.size());
  for (const auto node_id : canopen_node_ids) {
    if (node_id < 1 || node_id > 127) {
      throw std::invalid_argument("CANopen node ID must be in the range 1..127");
    }
    const auto typed_node_id = static_cast<std::uint8_t>(node_id);
    if (!unique_node_ids.insert(typed_node_id).second) {
      throw std::invalid_argument("CANopen node IDs must be unique");
    }
    typed_node_ids.push_back(typed_node_id);
  }

  return DiagnosticsTopology(
    std::move(ethercat_axes),
    std::move(ethercat_sensors),
    static_cast<std::uint16_t>(ethercat_expected_responders),
    std::move(typed_node_ids));
}

DiagnosticsTopology::DiagnosticsTopology(
  std::vector<EthercatAxis> ethercat_axes,
  std::vector<EthercatSensor> ethercat_sensors,
  std::uint16_t ethercat_expected_responders,
  std::vector<std::uint8_t> canopen_node_ids)
: ethercat_axes_(std::move(ethercat_axes)),
  ethercat_sensors_(std::move(ethercat_sensors)),
  ethercat_expected_responders_(ethercat_expected_responders),
  canopen_node_ids_(std::move(canopen_node_ids))
{}

const std::vector<EthercatAxis> & DiagnosticsTopology::ethercat_axes() const noexcept
{
  return ethercat_axes_;
}

const std::vector<EthercatSensor> & DiagnosticsTopology::ethercat_sensors() const noexcept
{
  return ethercat_sensors_;
}

std::uint16_t DiagnosticsTopology::ethercat_expected_responders() const noexcept
{
  return ethercat_expected_responders_;
}

const std::vector<std::uint8_t> & DiagnosticsTopology::canopen_node_ids() const noexcept
{
  return canopen_node_ids_;
}

void DiagnosticsTopology::validate_hardware_id(const std::string & hardware_id) const
{
  if (hardware_id.empty()) {
    throw std::invalid_argument("diagnostic hardware_id must be nonempty");
  }
  for (const auto node_id : canopen_node_ids_) {
    if (hardware_id == std::to_string(node_id)) {
      throw std::invalid_argument(
              "diagnostic hardware_id must not equal a configured CANopen node ID");
    }
  }
}

}  // namespace rt_diagnostics
