#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "ethercat_manager/ec_master_async.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace x503_force_sensor
{
namespace
{
diagnostic_msgs::msg::KeyValue key(std::string name, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(name);
  result.value = std::move(value);
  return result;
}

std::uint32_t decodeUnsigned(const std::vector<std::uint8_t> & bytes)
{
  if (bytes.empty() || bytes.size() > sizeof(std::uint32_t)) {
    throw std::runtime_error("X503B SDO value has unsupported size");
  }
  std::uint32_t value = 0U;
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    value |= static_cast<std::uint32_t>(bytes[index]) << (8U * index);
  }
  return value;
}

}  // namespace

class X503SdoSnapshotNode final : public rclcpp::Node
{
public:
  X503SdoSnapshotNode()
  : Node("x503_sdo_snapshot")
  {
    const auto master_index = declare_parameter<std::int64_t>("master_index", 0);
    sensor_names_ = declare_parameter<std::vector<std::string>>(
      "sensor_names", std::vector<std::string>{});
    const auto slave_positions = declare_parameter<std::vector<std::int64_t>>(
      "slave_positions", std::vector<std::int64_t>{});
    calibration_topic_ = declare_parameter<std::string>(
      "calibration_topic", "/rt_control/x503b/calibration");
    const auto retry_count = declare_parameter<std::int64_t>("retry_count", 20);
    const auto retry_period_ms = declare_parameter<std::int64_t>("retry_period_ms", 500);
    readback_config_ = declare_parameter<std::string>("readback_config", "");

    if (sensor_names_.size() != slave_positions.size() || sensor_names_.empty()) {
      throw std::invalid_argument("X503B sensor_names and slave_positions must have equal nonzero size");
    }
    if (
      master_index < 0 || master_index > 65535 || retry_count < 1 || retry_period_ms < 1 ||
      retry_count > std::numeric_limits<int>::max() ||
      retry_period_ms > std::numeric_limits<int>::max())
    {
      throw std::invalid_argument("invalid X503B SDO snapshot parameters");
    }
    master_index_ = static_cast<int>(master_index);
    retry_count_ = static_cast<int>(retry_count);
    retry_period_ms_ = static_cast<int>(retry_period_ms);
    for (const auto position : slave_positions) {
      if (position < 0 || position > 65534) {
        throw std::invalid_argument("X503B slave position is outside 0..65534");
      }
      slave_positions_.push_back(static_cast<int>(position));
    }
    loadReadbackConfig();

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(calibration_topic_, qos);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(retry_period_ms_),
      std::bind(&X503SdoSnapshotNode::tryRead, this));
  }

private:
  void loadReadbackConfig()
  {
    if (readback_config_.empty()) {
      throw std::invalid_argument("X503B readback_config is required");
    }
    const auto document = YAML::LoadFile(readback_config_);
    if (document["schema_version"].as<int>() != 1) {
      throw std::invalid_argument("unsupported X503B readback schema_version");
    }
    const auto readback = document["read_only_sdo"];
    engineering_unit_contract_ = document["engineering_unit_contract"].as<std::string>();
    validity_policy_ = document["validity_policy"].as<std::string>();
    valid_sample_codes_ = document["valid_sample_codes"].as<std::vector<std::int64_t>>();
    if (
      engineering_unit_contract_ != "unresolved" &&
      engineering_unit_contract_ != "force_N_torque_Nm")
    {
      throw std::invalid_argument("unsupported X503B engineering_unit_contract");
    }
    if (validity_policy_ != "unresolved" && validity_policy_ != "sample_codes_equal") {
      throw std::invalid_argument("unsupported X503B validity_policy");
    }
    if (validity_policy_ == "sample_codes_equal" && valid_sample_codes_.size() != 6U) {
      throw std::invalid_argument("sample_codes_equal requires six valid_sample_codes");
    }
    sdo_index_ = readback["index"].as<std::uint16_t>();
    decimal_subindices_ = readback["decimal_subindices"].as<std::vector<std::uint8_t>>();
    unit_subindices_ = readback["unit_subindices"].as<std::vector<std::uint8_t>>();
    if (readback["expected_unit_codes"]) {
      expected_unit_codes_ =
        readback["expected_unit_codes"].as<std::vector<std::uint32_t>>();
    }
    if (
      decimal_subindices_.size() != 6U || unit_subindices_.size() != 6U ||
      (!expected_unit_codes_.empty() && expected_unit_codes_.size() != 6U) ||
      (engineering_unit_contract_ == "force_N_torque_Nm" &&
      expected_unit_codes_.size() != 6U))
    {
      throw std::invalid_argument("X503B readback config must describe six channels");
    }
  }

  bool readUnsigned(
    ethercat_manager::EcMasterAsync & master, int slave_position,
    std::uint16_t index, std::uint8_t subindex, std::uint32_t & value)
  {
    ec_ioctl_slave_sdo_upload_t data{};
    std::array<std::uint8_t, sizeof(std::uint32_t)> target{};
    data.slave_position = static_cast<std::uint16_t>(slave_position);
    data.sdo_index = index;
    data.sdo_entry_subindex = subindex;
    data.target_size = target.size();
    data.target = target.data();
    master.sdo_upload(&data);
    if (data.data_size == 0U || data.data_size > target.size()) {
      return false;
    }
    value = decodeUnsigned(std::vector<std::uint8_t>(target.begin(), target.begin() + data.data_size));
    return true;
  }

  diagnostic_msgs::msg::DiagnosticStatus readSensor(
    ethercat_manager::EcMasterAsync & master, const std::string & sensor_name,
    int slave_position)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "/robot/rt_control/x503b/" + sensor_name + "/calibration";
    status.hardware_id = sensor_name;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "X503B read-only SDO snapshot unavailable";
    status.values.push_back(key("slave_position", std::to_string(slave_position)));
    status.values.push_back(key("snapshot_valid", "false"));
    status.values.push_back(key("engineering_unit_contract", engineering_unit_contract_));
    status.values.push_back(key("validity_policy", validity_policy_));

    std::array<std::uint32_t, 6U> decimals{};
    std::array<std::uint32_t, 6U> units{};
    try {
      for (std::size_t channel = 0; channel < 6U; ++channel) {
        if (!readUnsigned(master, slave_position, sdo_index_, decimal_subindices_[channel], decimals[channel])) {
          return status;
        }
        if (!readUnsigned(master, slave_position, sdo_index_, unit_subindices_[channel], units[channel])) {
          return status;
        }
        if (
          decimals[channel] > 9U ||
          (!expected_unit_codes_.empty() && units[channel] != expected_unit_codes_[channel]))
        {
          return status;
        }
      }
    } catch (const std::exception & error) {
      status.message = std::string("X503B SDO read failed: ") + error.what();
      return status;
    }

    status.level = engineering_unit_contract_ == "force_N_torque_Nm" &&
      validity_policy_ == "sample_codes_equal"
      ? diagnostic_msgs::msg::DiagnosticStatus::OK
      : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = engineering_unit_contract_ == "force_N_torque_Nm" &&
      validity_policy_ == "sample_codes_equal"
      ? "X503B unit/decimal snapshot valid; shadow Wrench enabled"
      : "X503B read-only snapshot valid; Wrench contract unresolved";
    status.values.clear();
    status.values.push_back(key("slave_position", std::to_string(slave_position)));
    status.values.push_back(key("snapshot_valid", "true"));
    status.values.push_back(key("engineering_unit_contract", engineering_unit_contract_));
    status.values.push_back(key("validity_policy", validity_policy_));
    for (std::size_t channel = 0; channel < 6U; ++channel) {
      status.values.push_back(key("decimal_" + std::to_string(channel + 1U), std::to_string(decimals[channel])));
      status.values.push_back(key("unit_" + std::to_string(channel + 1U), std::to_string(units[channel])));
      if (validity_policy_ == "sample_codes_equal") {
        status.values.push_back(
          key("valid_sample_code_" + std::to_string(channel + 1U),
            std::to_string(valid_sample_codes_[channel])));
      }
    }
    return status;
  }

  void tryRead()
  {
    if (attempts_ >= retry_count_) {
      timer_->cancel();
      return;
    }
    ++attempts_;
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    try {
      ethercat_manager::EcMasterAsync master(static_cast<std::uint16_t>(master_index_));
      master.open(ethercat_manager::EcMasterAsync::Read);
      for (std::size_t index = 0; index < sensor_names_.size(); ++index) {
        array.status.push_back(readSensor(master, sensor_names_[index], slave_positions_[index]));
      }
      master.close();
      const bool all_valid = std::all_of(
        array.status.begin(), array.status.end(),
        [](const auto & status) {
          return status.level != diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        });
      publisher_->publish(array);
      if (all_valid) {
        timer_->cancel();
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "X503B read-only SDO snapshot attempt %d failed: %s", attempts_, error.what());
    }
  }

  int master_index_{};
  std::vector<std::string> sensor_names_;
  std::vector<int> slave_positions_;
  std::string calibration_topic_;
  std::string engineering_unit_contract_;
  std::string validity_policy_;
  std::string readback_config_;
  std::uint16_t sdo_index_{};
  std::vector<std::uint8_t> decimal_subindices_;
  std::vector<std::uint8_t> unit_subindices_;
  std::vector<std::uint32_t> expected_unit_codes_;
  std::vector<std::int64_t> valid_sample_codes_;
  int retry_count_{};
  int retry_period_ms_{};
  int attempts_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace x503_force_sensor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<x503_force_sensor::X503SdoSnapshotNode>());
  } catch (const std::exception & error) {
    fprintf(stderr, "x503_sdo_snapshot: %s\n", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
