#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "rt_control_semantic_components/cia402_axis.hpp"
#include "rt_diagnostics/canopen_native_diagnostic.hpp"
#include "rt_diagnostics/diagnostic_summary.hpp"
#include "rt_diagnostics/diagnostics_topology.hpp"
#include "rt_diagnostics/dynamic_state_snapshot.hpp"
#include "rt_diagnostics/ethercat_master_diagnostic.hpp"

namespace rt_diagnostics
{
namespace
{
using SteadyClock = std::chrono::steady_clock;
constexpr auto kStaleTimeout = std::chrono::seconds(3);
constexpr char kEthercatSummaryName[] = "/robot/rt_control/ethercat/summary";
constexpr char kCanopenSummaryName[] = "/robot/rt_control/canopen/summary";
using rt_control_semantic_components::Cia402Axis;
using rt_control_semantic_components::Cia402State;

diagnostic_msgs::msg::KeyValue makeKeyValue(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = std::move(key);
  item.value = std::move(value);
  return item;
}

const char * cia402StateName(Cia402State state)
{
  switch (state) {
    case Cia402State::kNotReadyToSwitchOn:
      return "NotReadyToSwitchOn";
    case Cia402State::kSwitchOnDisabled:
      return "SwitchOnDisabled";
    case Cia402State::kReadyToSwitchOn:
      return "ReadyToSwitchOn";
    case Cia402State::kSwitchedOn:
      return "SwitchedOn";
    case Cia402State::kOperationEnabled:
      return "OperationEnabled";
    case Cia402State::kQuickStopActive:
      return "QuickStopActive";
    case Cia402State::kFaultReactionActive:
      return "FaultReactionActive";
    case Cia402State::kFault:
      return "Fault";
    case Cia402State::kUnknown:
      return "Unknown";
  }
  return "Unknown";
}

bool isFault(Cia402State state)
{
  return state == Cia402State::kFaultReactionActive || state == Cia402State::kFault;
}

bool toExactUnsigned(double value, std::uint16_t maximum, std::uint16_t & result)
{
  if (
    !std::isfinite(value) || value < 0.0 || value > static_cast<double>(maximum) ||
    std::trunc(value) != value)
  {
    return false;
  }
  result = static_cast<std::uint16_t>(value);
  return true;
}
}  // namespace

class RtDiagnosticsNode final : public rclcpp::Node
{
public:
  RtDiagnosticsNode()
  : Node("rt_diagnostics")
  {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    topology_.emplace(
      DiagnosticsTopology::create(
        declare_parameter<std::vector<std::string>>(
          "ethercat_joint_names", std::vector<std::string>{}, read_only),
        declare_parameter<std::vector<std::int64_t>>(
          "ethercat_ring_positions", std::vector<std::int64_t>{}, read_only),
        declare_parameter<std::vector<std::string>>(
          "ethercat_sensor_names", std::vector<std::string>{}, read_only),
        declare_parameter<std::vector<std::int64_t>>(
          "ethercat_sensor_ring_positions", std::vector<std::int64_t>{}, read_only),
        declare_parameter<std::int64_t>("ethercat_expected_responders", 0, read_only),
        declare_parameter<std::vector<std::int64_t>>(
          "canopen_node_ids", std::vector<std::int64_t>{}, read_only)));
    for (const auto node_id : topology_->canopen_node_ids()) {
      canopen_.emplace(std::to_string(node_id), NativeCanopenDiagnosticSnapshot{});
    }
    hardware_id_ = declare_parameter<std::string>("hardware_id", "robot-001", read_only);
    topology_->validate_hardware_id(hardware_id_);
    const auto dynamic_joint_states_topic = declare_parameter<std::string>(
      "dynamic_joint_states_topic", "/rt_internal_state_broadcaster/dynamic_joint_states");
    dynamic_state_subscription_ = create_subscription<control_msgs::msg::DynamicJointState>(
      dynamic_joint_states_topic, rclcpp::QoS(10),
      std::bind(&RtDiagnosticsNode::onDynamicState, this, std::placeholders::_1));
    diagnostic_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", robot_interfaces_qos::diagnostic(),
      std::bind(&RtDiagnosticsNode::onDiagnostics, this, std::placeholders::_1));
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", robot_interfaces_qos::diagnostic());
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publishSnapshot();});
  }

private:
  void onDynamicState(const control_msgs::msg::DynamicJointState::SharedPtr message)
  {
    std::unordered_map<std::string, double> next_values;
    const bool snapshot_valid = replaceDynamicStateSnapshot(*message, next_values);
    std::lock_guard<std::mutex> lock(mutex_);
    values_ = std::move(next_values);
    dynamic_state_received_ = SteadyClock::now();
    has_dynamic_state_ = snapshot_valid;
  }

  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & status : message->status) {
      if (status.name == "/robot/rt_control/enable_manager") {
        for (const auto & value : status.values) {
          if (value.key == "state") {
            enable_manager_state_ = value.value;
          }
        }
        continue;
      }
      const auto configured_node = canopen_.find(status.hardware_id);
      if (configured_node == canopen_.end()) {
        continue;
      }
      NativeCanopenDiagnostic diagnostic;
      diagnostic.level = status.level;
      diagnostic.message = status.message;
      diagnostic.values.reserve(status.values.size());
      for (const auto & value : status.values) {
        diagnostic.values.push_back({value.key, value.value});
      }
      static_cast<void>(ingestConfiguredNativeCanopenDiagnostic(
        canopen_, status.hardware_id, std::move(diagnostic), SteadyClock::now()));
    }
  }

  bool getValue(
    const std::unordered_map<std::string, double> & values, const std::string & name,
    double & value) const
  {
    const auto item = values.find(name);
    if (item == values.end()) {
      return false;
    }
    value = item->second;
    return std::isfinite(value);
  }

  void publishSnapshot()
  {
    std::unordered_map<std::string, double> values;
    std::unordered_map<std::string, NativeCanopenDiagnosticSnapshot> canopen;
    SteadyClock::time_point received;
    bool has_dynamic_state = false;
    std::string enable_state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      values = values_;
      canopen = canopen_;
      received = dynamic_state_received_;
      has_dynamic_state = has_dynamic_state_;
      enable_state = enable_manager_state_;
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    const auto current_time = SteadyClock::now();
    const bool topic_stale = !has_dynamic_state || current_time - received > kStaleTimeout;

    double process_age_ms = 0.0;
    const bool has_process_age =
      getValue(values, "ethercat_domain/process_data_age_ms", process_age_ms);
    const bool process_stale =
      !has_process_age || process_age_ms<0.0 || process_age_ms>3000.0;
    diagnostic_msgs::msg::DiagnosticStatus master;
    master.name = "/robot/rt_control/ethercat/master";
    master.hardware_id = hardware_id_;
    double link_up = 0.0;
    double slaves_responding_value = 0.0;
    double wc_error_count = 0.0;
    const bool master_values_complete =
      getValue(values, "ethercat_master/link_up", link_up) &&
      getValue(values, "ethercat_master/slaves_responding", slaves_responding_value) &&
      getValue(values, "ethercat_master/wc_error_count", wc_error_count);
    EthercatMasterDiagnosticSample master_sample{
      process_age_ms, link_up, slaves_responding_value, wc_error_count,
      has_process_age && master_values_complete};
    const auto master_assessment = evaluateEthercatMasterDiagnostic(
      master_sample, topic_stale, topology_->ethercat_expected_responders(),
      has_previous_wc_count_ ? std::optional<double>{previous_wc_error_count_} : std::nullopt);
    master.level = master_assessment.level;
    master.message = master_assessment.message;
    master.values.push_back(makeKeyValue("link", std::to_string(link_up)));
    master.values.push_back(
      makeKeyValue("slaves_responding", std::to_string(slaves_responding_value)));
    master.values.push_back(makeKeyValue("wc_error_count", std::to_string(wc_error_count)));
    master.values.push_back(
      makeKeyValue("process_data_age_ms", std::to_string(process_age_ms)));
    DiagnosticSummaryAccumulator ethercat_summary_accumulator{"EtherCAT topology healthy"};
    ethercat_summary_accumulator.observe(master.level, master.name, master.message);
    array.status.push_back(std::move(master));
    if (master_assessment.values_valid) {
      previous_wc_error_count_ = master_assessment.working_counter_error_count;
      has_previous_wc_count_ = true;
    }

    const bool expect_enabled = enable_state == "ENABLED";
    for (const auto & axis : topology_->ethercat_axes()) {
      const auto position = axis.ring_position;
      diagnostic_msgs::msg::DiagnosticStatus slave;
      slave.name = "/robot/rt_control/ethercat/slave_" + std::to_string(position);
      slave.hardware_id = hardware_id_;
      double al_state = 0.0;
      double status_word_value = 0.0;
      std::uint16_t al_state_value = 0U;
      std::uint16_t status_word = 0U;
      const bool raw_complete =
        getValue(values, "ethercat_slave_" + std::to_string(position) + "/al_state", al_state) &&
        getValue(values, axis.joint_name + "/status_word", status_word_value);
      const bool complete = raw_complete &&
        toExactUnsigned(al_state, std::numeric_limits<std::uint8_t>::max(), al_state_value) &&
        toExactUnsigned(
        status_word_value, std::numeric_limits<std::uint16_t>::max(), status_word);
      const auto cia402_state = Cia402Axis::decode_state(status_word);
      if (topic_stale || process_stale || !complete) {
        slave.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        slave.message = "EtherCAT slave snapshot is stale";
      } else if (isFault(cia402_state)) {
        slave.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        slave.message = "CiA402 fault";
      } else if (
        al_state_value != 0x08U ||
        (expect_enabled && (status_word & 0x006FU) != 0x0027U))
      {
        slave.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        slave.message = "EtherCAT slave is not in the expected state";
      } else {
        slave.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        slave.message = "EtherCAT slave state expected";
      }
      slave.values.push_back(makeKeyValue("al_state", std::to_string(al_state)));
      slave.values.push_back(makeKeyValue("cia402_state", cia402StateName(cia402_state)));
      slave.values.push_back(makeKeyValue("position", std::to_string(position)));
      ethercat_summary_accumulator.observe(slave.level, slave.name, slave.message);
      array.status.push_back(std::move(slave));
    }

    for (const auto & sensor : topology_->ethercat_sensors()) {
      const auto position = sensor.ring_position;
      diagnostic_msgs::msg::DiagnosticStatus slave;
      slave.name = "/robot/rt_control/ethercat/slave_" + std::to_string(position);
      slave.hardware_id = hardware_id_;
      double al_state = 0.0;
      double channel_1_raw = 0.0;
      std::uint16_t al_state_value = 0U;
      const bool complete =
        getValue(
        values, "ethercat_slave_" + std::to_string(position) + "/al_state", al_state) &&
        getValue(values, sensor.sensor_name + "/channel_1_raw", channel_1_raw) &&
        toExactUnsigned(al_state, std::numeric_limits<std::uint8_t>::max(), al_state_value);
      if (topic_stale || process_stale || !complete) {
        slave.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        slave.message = "X503 process-data snapshot is stale";
      } else if (al_state_value != 0x08U) {
        slave.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        slave.message = "X503 is not in OP";
      } else {
        slave.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        slave.message = "X503 raw input available";
      }
      slave.values.push_back(makeKeyValue("al_state", std::to_string(al_state)));
      slave.values.push_back(makeKeyValue("position", std::to_string(position)));
      slave.values.push_back(makeKeyValue("channel_1_raw", std::to_string(channel_1_raw)));
      ethercat_summary_accumulator.observe(slave.level, slave.name, slave.message);
      array.status.push_back(std::move(slave));
    }

    const auto ethercat_summary_result = ethercat_summary_accumulator.result();
    diagnostic_msgs::msg::DiagnosticStatus ethercat_summary;
    ethercat_summary.name = kEthercatSummaryName;
    ethercat_summary.hardware_id = hardware_id_;
    ethercat_summary.level = ethercat_summary_result.level;
    ethercat_summary.message = ethercat_summary_result.message;
    ethercat_summary.values.push_back(
      makeKeyValue(
        "configured_axes", std::to_string(topology_->ethercat_axes().size())));
    ethercat_summary.values.push_back(
      makeKeyValue(
        "configured_sensors", std::to_string(topology_->ethercat_sensors().size())));
    ethercat_summary.values.push_back(
      makeKeyValue(
        "expected_responders", std::to_string(topology_->ethercat_expected_responders())));
    array.status.push_back(std::move(ethercat_summary));

    DiagnosticSummaryAccumulator canopen_summary_accumulator{"CANopen topology healthy"};
    for (const auto node_id : topology_->canopen_node_ids()) {
      const auto hardware_id = std::to_string(node_id);
      const auto snapshot = canopen.find(hardware_id);
      diagnostic_msgs::msg::DiagnosticStatus normalized;
      normalized.name = "/robot/rt_control/canopen/node_" + hardware_id;
      normalized.hardware_id = hardware_id_;
      if (
        snapshot == canopen.end() || !snapshot->second.valid ||
        current_time - snapshot->second.received > kStaleTimeout)
      {
        normalized.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        normalized.message = "ros2_canopen native diagnostics are stale";
      } else {
        const auto assessment = evaluateNativeCanopenDiagnostic(snapshot->second.diagnostic);
        normalized.level = assessment.level;
        normalized.message = assessment.message;
        normalized.values.reserve(snapshot->second.diagnostic.values.size() + 1U);
        for (const auto & value : snapshot->second.diagnostic.values) {
          normalized.values.push_back(makeKeyValue(value.key, value.value));
        }
      }
      normalized.values.push_back(
        makeKeyValue("source_hardware_id", hardware_id));
      canopen_summary_accumulator.observe(normalized.level, normalized.name, normalized.message);
      array.status.push_back(std::move(normalized));
    }

    const auto canopen_summary_result = canopen_summary_accumulator.result();
    diagnostic_msgs::msg::DiagnosticStatus canopen_summary;
    canopen_summary.name = kCanopenSummaryName;
    canopen_summary.hardware_id = hardware_id_;
    canopen_summary.level = canopen_summary_result.level;
    canopen_summary.message = canopen_summary_result.message;
    canopen_summary.values.push_back(
      makeKeyValue(
        "configured_nodes", std::to_string(topology_->canopen_node_ids().size())));
    array.status.push_back(std::move(canopen_summary));

    publisher_->publish(array);
  }

  std::mutex mutex_;
  std::unordered_map<std::string, double> values_;
  std::optional<DiagnosticsTopology> topology_;
  std::unordered_map<std::string, NativeCanopenDiagnosticSnapshot> canopen_;
  SteadyClock::time_point dynamic_state_received_{};
  bool has_dynamic_state_{false};
  std::string enable_manager_state_;
  double previous_wc_error_count_{0.0};
  bool has_previous_wc_count_{false};
  std::string hardware_id_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr
    dynamic_state_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rt_diagnostics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rt_diagnostics::RtDiagnosticsNode>());
  rclcpp::shutdown();
  return 0;
}
