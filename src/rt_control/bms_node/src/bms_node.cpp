#include "bms_node/node.hpp"

#include "bms_node/chinese_status.hpp"
#include "bms_node/protocol/can_transport.hpp"
#include "bms_node/protocol/protocol.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace bms_node
{
namespace
{

double monotonic_seconds() noexcept
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::chrono::nanoseconds seconds_to_duration(const double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>{seconds});
}

class BmsNode final : public rclcpp::Node
{
public:
  explicit BmsNode(const rclcpp::NodeOptions & options)
  : Node{"bms_node", options}
  {
    if (declare_parameter<std::string>("transport", "socketcan") != "socketcan") {
      throw std::invalid_argument{"bms_node requires transport=socketcan"};
    }
    can_interface_ = declare_parameter<std::string>("can_interface", "can1");
    const bool valid_interface_name =
      !can_interface_.empty() && can_interface_.size() < IFNAMSIZ &&
      std::all_of(
        can_interface_.begin(), can_interface_.end(),
        [](const char ch) {
          return std::isalnum(static_cast<unsigned char>(ch)) != 0 || ch == '_' || ch == '-' ||
                 ch == '.';
        }) &&
      can_interface_ != "." && can_interface_ != "..";
    if (!valid_interface_name) {
      throw std::invalid_argument{"invalid CAN interface name"};
    }
    expected_adapter_serial_ = declare_parameter<std::string>("expected_adapter_serial", "");
    const auto protocol = declare_parameter<std::string>("protocol", "golden_phoenix_v1_1");
    const auto bitrate = declare_parameter<std::int64_t>("can_bitrate", 250000);
    if (protocol != "golden_phoenix_v1_1" || bitrate != 250000) {
      throw std::invalid_argument{"only Golden Phoenix V1.1 at 250 kbit/s is supported"};
    }
    primary_address_ = address_parameter("primary_bms_address", kDefaultBmsAddress);
    secondary_address_ = address_parameter("secondary_bms_address", kSecondaryBmsAddress);
    host_address_ = address_parameter("host_address", kDefaultHostAddress);
    if (primary_address_ == secondary_address_ || primary_address_ == host_address_ ||
      secondary_address_ == host_address_)
    {
      throw std::invalid_argument{"primary, secondary and host CAN addresses must differ"};
    }
    const auto order = parse_byte_order(
      declare_parameter<std::string>("multi_byte_order", "auto"));
    request_period_s_ = positive_parameter("request_period_s", 0.2);
    receive_timeout_s_ = positive_parameter("receive_timeout_s", 0.5);
    reconnect_period_s_ = positive_parameter("reconnect_period_s", 2.0);
    frame_timeout_s_ = positive_parameter("frame_timeout_s", 3.0);
    const double publish_period_s = positive_parameter("publish_period_s", 5.0);
    const auto primary_battery_topic =
      string_parameter("primary_battery_state_topic", "/battery_state");
    const auto secondary_battery_topic =
      string_parameter("secondary_battery_state_topic", "/battery_state/secondary");
    const auto primary_status_topic =
      string_parameter("primary_status_chinese_topic", "/bms/primary/status_chinese");
    const auto secondary_status_topic =
      string_parameter("secondary_status_chinese_topic", "/bms/secondary/status_chinese");
    const auto primary_fault_topic =
      string_parameter("primary_fault_chinese_topic", "/bms/primary/fault_chinese");
    const auto secondary_fault_topic =
      string_parameter("secondary_fault_chinese_topic", "/bms/secondary/fault_chinese");
    validate_distinct_names(
      {primary_battery_topic, secondary_battery_topic, primary_status_topic, secondary_status_topic,
       primary_fault_topic, secondary_fault_topic},
      "BMS topics");

    status_options_.mos_closed_raw = raw_mapping_parameter("mos_closed_raw", -1);
    status_options_.mos_open_raw = raw_mapping_parameter("mos_open_raw", -1);
    if (
      (status_options_.mos_closed_raw < 0) != (status_options_.mos_open_raw < 0) ||
      (status_options_.mos_closed_raw >= 0 &&
       status_options_.mos_closed_raw == status_options_.mos_open_raw)) {
      throw std::invalid_argument{
        "mos_closed_raw and mos_open_raw must both be -1 or distinct bytes"};
    }

    primary_decoder_ = std::make_unique<GoldenPhoenixPack>(order, primary_address_, host_address_);
    secondary_decoder_ = std::make_unique<GoldenPhoenixPack>(
      order, secondary_address_, host_address_);
    request_ids_ = make_dual_poll_ids(primary_address_, secondary_address_, host_address_);
    constexpr std::array<std::uint8_t, 7U> data_ids{
      kTotalStatusDataId, kCellExtremesDataId, kCellTemperatureDataId, kMosStatusDataId,
      kPackInfoDataId, kCellVoltagesDataId, kFaultDataId};
    for (std::size_t index = 0U; index < data_ids.size(); ++index) {
      response_ids_[index] = make_can_id(data_ids[index], host_address_, primary_address_);
      response_ids_[index + data_ids.size()] =
        make_can_id(data_ids[index], host_address_, secondary_address_);
    }

    primary_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
      primary_battery_topic, robot_interfaces_qos::state());
    secondary_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
      secondary_battery_topic, robot_interfaces_qos::state());
    primary_status_publisher_ =
      create_publisher<std_msgs::msg::String>(primary_status_topic, robot_interfaces_qos::state());
    secondary_status_publisher_ = create_publisher<std_msgs::msg::String>(
      secondary_status_topic, robot_interfaces_qos::state());
    primary_fault_publisher_ =
      create_publisher<std_msgs::msg::String>(primary_fault_topic, robot_interfaces_qos::state());
    secondary_fault_publisher_ =
      create_publisher<std_msgs::msg::String>(secondary_fault_topic, robot_interfaces_qos::state());

    publish_timer_ =
      create_wall_timer(seconds_to_duration(publish_period_s), [this]() { publish(); });
    reader_thread_ = std::thread{[this]() {reader_loop();}};
    RCLCPP_INFO(
      get_logger(),
      "BMS monitoring on %s for addresses 0x%02x/0x%02x; relay control is owned by plc_io_modbus",
      can_interface_.c_str(), primary_address_, secondary_address_);
  }

  ~BmsNode() override
  {
    stop_.store(true);
    wait_condition_.notify_all();
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
  }

private:
  double positive_parameter(const std::string & name, const double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument{name + " must be finite and positive"};
    }
    return value;
  }

  std::string string_parameter(const std::string & name, const std::string & default_value)
  {
    const auto value = declare_parameter<std::string>(name, default_value);
    if (value.empty()) {
      throw std::invalid_argument{name + " must not be empty"};
    }
    return value;
  }

  std::uint8_t address_parameter(const std::string & name, const std::uint8_t default_value)
  {
    const auto value = declare_parameter<std::int64_t>(name, default_value);
    if (value < 0 || value > std::numeric_limits<std::uint8_t>::max()) {
      throw std::invalid_argument{name + " must be in [0, 255]"};
    }
    return static_cast<std::uint8_t>(value);
  }

  int raw_mapping_parameter(const std::string & name, const int default_value)
  {
    const auto value = declare_parameter<std::int64_t>(name, default_value);
    if (value < -1 || value > 255) {
      throw std::invalid_argument{name + " must be -1 or in [0, 255]"};
    }
    return static_cast<int>(value);
  }

  static void validate_distinct_names(
    const std::initializer_list<std::string> names, const std::string & group)
  {
    const std::vector<std::string> values{names};
    for (std::size_t i = 0U; i < values.size(); ++i) {
      for (std::size_t j = i + 1U; j < values.size(); ++j) {
        if (values[i] == values[j]) {
          throw std::invalid_argument{group + " must be pairwise distinct"};
        }
      }
    }
  }

  int connect_can()
  {
    if (!expected_adapter_serial_.empty()) {
      std::ifstream serial_file{"/sys/class/net/" + can_interface_ + "/device/../serial"};
      std::string actual_serial;
      if (!std::getline(serial_file, actual_serial) || actual_serial != expected_adapter_serial_) {
        RCLCPP_WARN(
          get_logger(), "CAN interface %s does not match configured adapter serial",
          can_interface_.c_str());
        return -1;
      }
    }
    const unsigned int index = if_nametoindex(can_interface_.c_str());
    if (index == 0U) {
      RCLCPP_WARN(get_logger(), "CAN interface %s is unavailable", can_interface_.c_str());
      return -1;
    }
    const int descriptor = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, CAN_RAW);
    if (descriptor < 0) {
      RCLCPP_WARN(get_logger(), "CAN socket open failed: %s", std::strerror(errno));
      return -1;
    }
    // Receive the dedicated BMS bus and apply the exact response whitelist below.
    // This also remains reliable across gs_usb interface down/up and rename cycles.
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(index);
    if (::bind(descriptor, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0) {
      RCLCPP_WARN(get_logger(), "CAN bind failed: %s", std::strerror(errno));
      ::close(descriptor);
      return -1;
    }
    RCLCPP_INFO(get_logger(), "connected to %s", can_interface_.c_str());
    return descriptor;
  }

  void wait_until(const std::chrono::steady_clock::time_point deadline)
  {
    std::unique_lock<std::mutex> lock{wait_mutex_};
    wait_condition_.wait_until(lock, deadline, [this]() {return stop_.load();});
  }

  void reader_loop()
  {
    int descriptor = -1;
    std::size_t request_index = 0U;
    auto next_connect = std::chrono::steady_clock::now();
    auto next_request = next_connect;
    const auto request_spacing = seconds_to_duration(
      request_period_s_ / static_cast<double>(request_ids_.size()));
    auto disconnect = [&]() {
        ::close(descriptor);
        descriptor = -1;
        next_connect = std::chrono::steady_clock::now() +
          seconds_to_duration(reconnect_period_s_);
        RCLCPP_WARN(get_logger(), "BMS CAN connection lost");
      };
    while (!stop_.load()) {
      const auto now = std::chrono::steady_clock::now();
      if (descriptor < 0 && now >= next_connect) {
        descriptor = connect_can();
        next_connect = now + seconds_to_duration(reconnect_period_s_);
        request_index = 0U;
        next_request = now;
      }
      if (descriptor < 0) {
        wait_until(next_connect);
        continue;
      }
      if (now >= next_request) {
        const can_frame query = make_query_frame(request_ids_[request_index]);
        if (::write(descriptor, &query, sizeof(query)) != static_cast<ssize_t>(sizeof(query))) {
          RCLCPP_WARN(get_logger(), "BMS query failed: %s", std::strerror(errno));
          disconnect();
          continue;
        }
        request_index = (request_index + 1U) % request_ids_.size();
        next_request = now + request_spacing;
      }
      const double remaining_s =
        std::chrono::duration<double>(next_request - std::chrono::steady_clock::now()).count();
      const double wait_s = std::max(0.001, std::min(receive_timeout_s_, remaining_s));
      pollfd event{descriptor, POLLIN, 0};
      const int result = ::poll(&event, 1, static_cast<int>(std::ceil(wait_s * 1000.0)));
      if (result == 0 || (result < 0 && errno == EINTR)) {
        continue;
      }
      if (result < 0 || (event.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        disconnect();
        continue;
      }
      if ((event.revents & POLLIN) == 0) {
        continue;
      }
      can_frame frame{};
      if (::read(descriptor, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
        disconnect();
        continue;
      }
      const std::uint8_t * data = nullptr;
      std::size_t size = 0U;
      bool valid = false;
      for (const auto response_id : response_ids_) {
        if (decode_response_frame(frame, response_id, data, size)) {
          valid = true;
          break;
        }
      }
      if (!valid) {
        continue;
      }
      const std::uint32_t id = frame.can_id & CAN_EFF_MASK;
      const double stamp = monotonic_seconds();
      std::lock_guard<std::mutex> lock{state_mutex_};
      static_cast<void>(primary_decoder_->ingest(primary_, id, data, size, stamp));
      static_cast<void>(secondary_decoder_->ingest(secondary_, id, data, size, stamp));
    }
    if (descriptor >= 0) {
      ::close(descriptor);
    }
  }

  void publish_pack(
    const PackState & pack, const std::uint8_t address,
    const rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr & battery_publisher,
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & status_publisher,
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & fault_publisher,
    const double timestamp)
  {
    const bool total_fresh = pack.sample.is_fresh(timestamp, frame_timeout_s_);
    const bool fault_fresh = pack.fault_frame_s && timestamp >= *pack.fault_frame_s &&
                             timestamp - *pack.fault_frame_s <= frame_timeout_s_;
    const bool temperature_fresh = pack.temperature_frame_s &&
                                   timestamp >= *pack.temperature_frame_s &&
                                   timestamp - *pack.temperature_frame_s <= frame_timeout_s_;
    const auto values = project_battery_values(pack.sample, timestamp, frame_timeout_s_);
    const bool present =
      total_fresh && std::isfinite(values.voltage_v) && std::isfinite(values.soc_fraction);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    sensor_msgs::msg::BatteryState message;
    message.header.stamp = now();
    message.header.frame_id = can_interface_;
    std::array<char, 5U> label{};
    std::snprintf(label.data(), label.size(), "0x%02x", address);
    message.serial_number = label.data();
    message.voltage = static_cast<float>(present ? values.voltage_v : nan);
    message.current = static_cast<float>(present && pack.sample.current_a ?
      *pack.sample.current_a : nan);
    message.percentage = static_cast<float>(present ? values.soc_fraction : nan);
    message.temperature = static_cast<float>(
      present && temperature_fresh && pack.max_cell_temperature_c ? *pack.max_cell_temperature_c
                                                                  : nan);
    message.charge = static_cast<float>(nan);
    message.capacity = static_cast<float>(nan);
    message.design_capacity = static_cast<float>(nan);
    message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    if (present && pack.operating_state_raw == 1U) {
      message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } else if (present && pack.operating_state_raw == 2U) {
      message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    }
    message.power_supply_health = fault_fresh && pack.has_fault() ?
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE :
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    message.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    message.present = present;
    battery_publisher->publish(message);

    std_msgs::msg::String status;
    status.data =
      format_chinese_status(pack, address, timestamp, frame_timeout_s_, status_options_);
    status_publisher->publish(status);
    std_msgs::msg::String fault;
    fault.data = format_chinese_fault(pack, address, timestamp, frame_timeout_s_);
    fault_publisher->publish(fault);
  }

  void publish()
  {
    PackState primary;
    PackState secondary;
    {
      std::lock_guard<std::mutex> lock{state_mutex_};
      primary = primary_;
      secondary = secondary_;
    }
    const double stamp = monotonic_seconds();
    publish_pack(
      primary, primary_address_, primary_publisher_, primary_status_publisher_,
      primary_fault_publisher_, stamp);
    publish_pack(
      secondary, secondary_address_, secondary_publisher_, secondary_status_publisher_,
      secondary_fault_publisher_, stamp);
  }

  std::string can_interface_;
  std::string expected_adapter_serial_;
  std::uint8_t primary_address_{kDefaultBmsAddress};
  std::uint8_t secondary_address_{kSecondaryBmsAddress};
  std::uint8_t host_address_{kDefaultHostAddress};
  double request_period_s_{0.2};
  double receive_timeout_s_{0.5};
  double reconnect_period_s_{2.0};
  double frame_timeout_s_{3.0};
  ChineseStatusOptions status_options_;
  std::array<std::uint32_t, 14U> request_ids_{};
  std::array<std::uint32_t, 14U> response_ids_{};
  std::unique_ptr<GoldenPhoenixPack> primary_decoder_;
  std::unique_ptr<GoldenPhoenixPack> secondary_decoder_;
  PackState primary_;
  PackState secondary_;
  std::mutex state_mutex_;
  std::atomic<bool> stop_{false};
  std::mutex wait_mutex_;
  std::condition_variable wait_condition_;
  std::thread reader_thread_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr primary_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr secondary_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr primary_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr secondary_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr primary_fault_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr secondary_fault_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace

std::shared_ptr<rclcpp::Node> make_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<BmsNode>(options);
}

}  // namespace bms_node
