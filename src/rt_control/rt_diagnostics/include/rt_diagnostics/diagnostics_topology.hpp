#ifndef RT_DIAGNOSTICS__DIAGNOSTICS_TOPOLOGY_HPP_
#define RT_DIAGNOSTICS__DIAGNOSTICS_TOPOLOGY_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace rt_diagnostics
{

struct EthercatAxis
{
  std::string joint_name;
  std::uint16_t ring_position;
};

struct EthercatSensor
{
  std::string sensor_name;
  std::uint16_t ring_position;
};

class DiagnosticsTopology
{
public:
  static DiagnosticsTopology create(
    std::vector<std::string> ethercat_joint_names,
    const std::vector<std::int64_t> & ethercat_ring_positions,
    std::vector<std::string> ethercat_sensor_names,
    const std::vector<std::int64_t> & ethercat_sensor_ring_positions,
    std::int64_t ethercat_expected_responders,
    const std::vector<std::int64_t> & canopen_node_ids);

  const std::vector<EthercatAxis> & ethercat_axes() const noexcept;
  const std::vector<EthercatSensor> & ethercat_sensors() const noexcept;
  std::uint16_t ethercat_expected_responders() const noexcept;
  const std::vector<std::uint8_t> & canopen_node_ids() const noexcept;
  void validate_hardware_id(const std::string & hardware_id) const;

private:
  DiagnosticsTopology(
    std::vector<EthercatAxis> ethercat_axes,
    std::vector<EthercatSensor> ethercat_sensors,
    std::uint16_t ethercat_expected_responders,
    std::vector<std::uint8_t> canopen_node_ids);

  std::vector<EthercatAxis> ethercat_axes_;
  std::vector<EthercatSensor> ethercat_sensors_;
  std::uint16_t ethercat_expected_responders_;
  std::vector<std::uint8_t> canopen_node_ids_;
};

}  // namespace rt_diagnostics

#endif  // RT_DIAGNOSTICS__DIAGNOSTICS_TOPOLOGY_HPP_
