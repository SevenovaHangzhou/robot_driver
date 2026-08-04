#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rt_diagnostics
{
namespace
{
using SteadyClock = std::chrono::steady_clock;
constexpr auto kStaleTimeout = std::chrono::seconds(3);
constexpr std::array<int, 14> kRingPositions = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15};
constexpr std::array<const char *, 14> kJointNames = {
  "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5",
  "right_joint6", "left_joint1", "left_joint2", "left_joint3", "left_joint4",
  "left_joint5", "left_joint6", "turn", "updown"};

diagnostic_msgs::msg::KeyValue makeKeyValue(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = std::move(key);
  item.value = std::move(value);
  return item;
}

const char * cia402StateName(std::uint16_t status_word)
{
  if ((status_word & 0x004FU) == 0x0000U) {return "NotReadyToSwitchOn";}
  if ((status_word & 0x004FU) == 0x0040U) {return "SwitchOnDisabled";}
  if ((status_word & 0x006FU) == 0x0021U) {return "ReadyToSwitchOn";}
  if ((status_word & 0x006FU) == 0x0023U) {return "SwitchedOn";}
  if ((status_word & 0x006FU) == 0x0027U) {return "OperationEnabled";}
  if ((status_word & 0x006FU) == 0x0007U) {return "QuickStopActive";}
  if ((status_word & 0x004FU) == 0x000FU) {return "FaultReactionActive";}
  if ((status_word & 0x004FU) == 0x0008U) {return "Fault";}
  return "Unknown";
}

bool isFault(std::uint16_t status_word)
{
  return (status_word & 0x004FU) == 0x000FU || (status_word & 0x004FU) == 0x0008U;
}
}  // namespace

class RtDiagnosticsNode final : public rclcpp::Node
{
public:
  RtDiagnosticsNode()
  : Node("rt_diagnostics")
  {
    hardware_id_ = declare_parameter<std::string>("hardware_id", "robot-001");
    const auto dynamic_joint_states_topic = declare_parameter<std::string>(
      "dynamic_joint_states_topic", "/rt_internal_state_broadcaster/dynamic_joint_states");
    dynamic_state_subscription_ = create_subscription<control_msgs::msg::DynamicJointState>(
      dynamic_joint_states_topic, rclcpp::QoS(10),
      std::bind(&RtDiagnosticsNode::onDynamicState, this, std::placeholders::_1));
    diagnostic_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(50),
      std::bind(&RtDiagnosticsNode::onDiagnostics, this, std::placeholders::_1));
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publishSnapshot();});
  }

private:
  struct CanopenSnapshot
  {
    CanopenSnapshot()
    : status()
    {}

    diagnostic_msgs::msg::DiagnosticStatus status;
    SteadyClock::time_point received{};
    bool valid{false};
  };

  void onDynamicState(const control_msgs::msg::DynamicJointState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t joint = 0; joint < message->joint_names.size(); ++joint) {
      if (joint >= message->interface_values.size()) {
        break;
      }
      const auto & interfaces = message->interface_values[joint];
      for (std::size_t item = 0; item < interfaces.interface_names.size(); ++item) {
        if (item >= interfaces.values.size()) {
          break;
        }
        values_[message->joint_names[joint] + "/" + interfaces.interface_names[item]] =
          interfaces.values[item];
      }
    }
    dynamic_state_received_ = SteadyClock::now();
    has_dynamic_state_ = true;
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
      if (status.hardware_id != "2" && status.hardware_id != "3") {
        continue;
      }
      bool has_native_state = false;
      for (const auto & value : status.values) {
        has_native_state = has_native_state || value.key == "nmt_state" ||
          value.key == "emcy_state" || value.key == "cia402_state";
      }
      if (!has_native_state) {
        continue;
      }
      const std::size_t index = static_cast<std::size_t>(std::stoi(status.hardware_id) - 2);
      canopen_[index].status = status;
      canopen_[index].received = SteadyClock::now();
      canopen_[index].valid = true;
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
    std::array<CanopenSnapshot, 2> canopen;
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
    const bool process_stale = !has_process_age || process_age_ms > 3000.0;

    diagnostic_msgs::msg::DiagnosticStatus master;
    master.name = "/robot/rt_control/ethercat/master";
    master.hardware_id = hardware_id_;
    double link_up = 0.0;
    double slaves_responding = 0.0;
    double wc_error_count = 0.0;
    const bool master_complete =
      getValue(values, "ethercat_master/link_up", link_up) &&
      getValue(values, "ethercat_master/slaves_responding", slaves_responding) &&
      getValue(values, "ethercat_master/wc_error_count", wc_error_count);
    if (topic_stale || process_stale || !master_complete) {
      master.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      master.message = "EtherCAT process-data snapshot is stale";
    } else if (link_up < 0.5) {
      master.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      master.message = "EtherCAT link is down";
    } else if (
      static_cast<int>(slaves_responding) != 16 ||
      (has_previous_wc_count_ && wc_error_count > previous_wc_error_count_))
    {
      master.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      master.message = "EtherCAT responder count or working counter changed";
    } else {
      master.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      master.message = "EtherCAT master healthy";
    }
    master.values.push_back(makeKeyValue("link", std::to_string(link_up)));
    master.values.push_back(
      makeKeyValue("slaves_responding", std::to_string(slaves_responding)));
    master.values.push_back(makeKeyValue("wc_error_count", std::to_string(wc_error_count)));
    master.values.push_back(
      makeKeyValue("process_data_age_ms", std::to_string(process_age_ms)));
    array.status.push_back(std::move(master));
    if (master_complete) {
      previous_wc_error_count_ = wc_error_count;
      has_previous_wc_count_ = true;
    }

    const bool expect_enabled = enable_state == "ENABLED";
    for (std::size_t axis = 0; axis < kRingPositions.size(); ++axis) {
      const int position = kRingPositions[axis];
      diagnostic_msgs::msg::DiagnosticStatus slave;
      slave.name = "/robot/rt_control/ethercat/slave_" + std::to_string(position);
      slave.hardware_id = hardware_id_;
      double al_state = 0.0;
      double status_word_value = 0.0;
      const bool complete =
        getValue(values, "ethercat_slave_" + std::to_string(position) + "/al_state", al_state) &&
        getValue(values, std::string(kJointNames[axis]) + "/status_word", status_word_value);
      const auto status_word = static_cast<std::uint16_t>(status_word_value);
      if (topic_stale || process_stale || !complete) {
        slave.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        slave.message = "EtherCAT slave snapshot is stale";
      } else if (isFault(status_word)) {
        slave.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        slave.message = "CiA402 fault";
      } else if (
        static_cast<int>(al_state) != 0x08 ||
        (expect_enabled && (status_word & 0x006FU) != 0x0027U))
      {
        slave.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        slave.message = "EtherCAT slave is not in the expected state";
      } else {
        slave.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        slave.message = "EtherCAT slave state expected";
      }
      slave.values.push_back(makeKeyValue("al_state", std::to_string(al_state)));
      slave.values.push_back(makeKeyValue("cia402_state", cia402StateName(status_word)));
      slave.values.push_back(makeKeyValue("position", std::to_string(position)));
      array.status.push_back(std::move(slave));
    }

    for (std::size_t node = 0; node < canopen.size(); ++node) {
      diagnostic_msgs::msg::DiagnosticStatus normalized;
      normalized.name = "/robot/rt_control/canopen/node_" + std::to_string(node + 2U);
      normalized.hardware_id = hardware_id_;
      if (!canopen[node].valid || current_time - canopen[node].received > kStaleTimeout) {
        normalized.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        normalized.message = "ros2_canopen native diagnostics are stale";
      } else {
        normalized.level = canopen[node].status.level;
        normalized.message = canopen[node].status.message;
        normalized.values = canopen[node].status.values;
      }
      normalized.values.push_back(
        makeKeyValue("source_hardware_id", std::to_string(node + 2U)));
      array.status.push_back(std::move(normalized));
    }

    publisher_->publish(array);
  }

  std::mutex mutex_;
  std::unordered_map<std::string, double> values_;
  std::array<CanopenSnapshot, 2> canopen_{};
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
