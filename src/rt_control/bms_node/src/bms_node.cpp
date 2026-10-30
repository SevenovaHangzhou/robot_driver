#include "bms_node/node.hpp"

#include "bms_node/protocol/can_transport.hpp"
#include "bms_node/dual_battery_controller.hpp"
#include "bms_node/protocol/protocol.hpp"
#include "infra_socketcan/socket.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include <net/if.h>

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
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

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
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    expected_adapter_serial_ = declare_parameter<std::string>(
      "expected_adapter_serial", "");
    const auto protocol = declare_parameter<std::string>(
      "protocol", "golden_phoenix_v1_1");
    const auto bitrate = declare_parameter<std::int64_t>("can_bitrate", 250000);
    if (protocol != "golden_phoenix_v1_1" || bitrate != 250000) {
      throw std::invalid_argument{"only Golden Phoenix V1.1 at 250 kbit/s is supported"};
    }
    automatic_discharge_control_ = declare_parameter<bool>("automatic_discharge_control", false);
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
    const auto topic = declare_parameter<std::string>(
      "battery_state_topic", "/battery_state");
    const auto secondary_topic = declare_parameter<std::string>(
      "secondary_battery_state_topic", "/battery_state/secondary");
    const double max_join_voltage_delta_v = declare_parameter<double>(
      "max_join_voltage_delta_v", -1.0);
    const double max_trip_voltage_delta_v = declare_parameter<double>(
      "max_trip_voltage_delta_v", -1.0);
    const double max_join_current_a = declare_parameter<double>(
      "max_join_current_a", -1.0);
    const double min_cell_voltage_v = declare_parameter<double>(
      "min_cell_voltage_v", -1.0);
    const double max_cell_voltage_v = declare_parameter<double>(
      "max_cell_voltage_v", -1.0);
    const double min_cell_temperature_c = declare_parameter<double>(
      "min_cell_temperature_c", -1000.0);
    const double max_cell_temperature_c = declare_parameter<double>(
      "max_cell_temperature_c", -1000.0);
    const double join_stable_s = declare_parameter<double>("join_stable_s", -1.0);
    const double observation_s = declare_parameter<double>("secondary_observation_s", -1.0);
    const double status_timeout_s = declare_parameter<double>(
      "control_status_timeout_s", -1.0);
    const double command_timeout_s = declare_parameter<double>("command_timeout_s", -1.0);
    const auto mos_on = declare_parameter<std::int64_t>("discharge_mos_on_raw", -1);
    const auto mos_off = declare_parameter<std::int64_t>("discharge_mos_off_raw", -1);
    const auto relay_command_topic = declare_parameter<std::string>("relay_command_topic", "");
    const auto relay_feedback_topic = declare_parameter<std::string>("relay_feedback_topic", "");
    const auto loads_stop_topic = declare_parameter<std::string>("loads_stop_request_topic", "");
    const auto loads_feedback_topic = declare_parameter<std::string>(
      "loads_stopped_feedback_topic", "");
    const bool valid_interface_name = !can_interface_.empty() &&
      can_interface_.size() < IFNAMSIZ &&
      std::all_of(can_interface_.begin(), can_interface_.end(), [](const char ch) {
        return std::isalnum(static_cast<unsigned char>(ch)) != 0 ||
               ch == '_' || ch == '-' || ch == '.';
      }) && can_interface_ != "." && can_interface_ != "..";
    if (!valid_interface_name ||
      topic.empty() || secondary_topic.empty() || topic == secondary_topic)
    {
      throw std::invalid_argument{"invalid CAN interface or battery state topics"};
    }
    if (automatic_discharge_control_) {
      const bool valid_topics = !relay_command_topic.empty() && !relay_feedback_topic.empty() &&
        !loads_stop_topic.empty() && !loads_feedback_topic.empty() &&
        relay_command_topic != relay_feedback_topic &&
        loads_stop_topic != loads_feedback_topic &&
        relay_command_topic != loads_stop_topic;
      if (!valid_topics || mos_on < 0 || mos_on > 255 || mos_off < 0 || mos_off > 255 ||
        mos_on == mos_off)
      {
        throw std::invalid_argument{"automatic discharge requires distinct topics and MOS values"};
      }
      controller_ = std::make_unique<DualBatteryController>(DischargeSettings{
        status_timeout_s, command_timeout_s, observation_s, join_stable_s,
        max_join_voltage_delta_v, max_trip_voltage_delta_v, max_join_current_a,
        min_cell_voltage_v, max_cell_voltage_v,
        min_cell_temperature_c, max_cell_temperature_c,
        static_cast<std::uint8_t>(mos_on), static_cast<std::uint8_t>(mos_off)});
    }

    primary_decoder_ = std::make_unique<GoldenPhoenixPack>(
      order, primary_address_, host_address_);
    secondary_decoder_ = std::make_unique<GoldenPhoenixPack>(
      order, secondary_address_, host_address_);
    request_ids_ = make_dual_poll_ids(
      primary_address_, secondary_address_, host_address_);
    constexpr std::array<std::uint8_t, 7U> data_ids{
      kTotalStatusDataId, kCellExtremesDataId, kCellTemperatureDataId, kMosStatusDataId,
      kPackInfoDataId, kCellVoltagesDataId, kFaultDataId};
    for (std::size_t index = 0U; index < data_ids.size(); ++index) {
      response_ids_[index] = make_can_id(data_ids[index], host_address_, primary_address_);
      response_ids_[index + data_ids.size()] =
        make_can_id(data_ids[index], host_address_, secondary_address_);
    }
    response_ids_[14U] = make_can_id(kDischargeMosDataId, host_address_, primary_address_);
    response_ids_[15U] = make_can_id(kDischargeMosDataId, host_address_, secondary_address_);

    primary_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
      topic, robot_interfaces_qos::state());
    secondary_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
      secondary_topic, robot_interfaces_qos::state());
    if (automatic_discharge_control_) {
      relay_command_publisher_ = create_publisher<std_msgs::msg::Bool>(relay_command_topic, 10);
      loads_stop_publisher_ = create_publisher<std_msgs::msg::Bool>(loads_stop_topic, 10);
      relay_feedback_subscription_ = create_subscription<std_msgs::msg::Bool>(
        relay_feedback_topic, 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
          std::lock_guard<std::mutex> lock{feedback_mutex_};
          relay_closed_ = message->data;
          relay_feedback_s_ = monotonic_seconds();
        });
      loads_feedback_subscription_ = create_subscription<std_msgs::msg::Bool>(
        loads_feedback_topic, 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
          std::lock_guard<std::mutex> lock{feedback_mutex_};
          loads_stopped_ = message->data;
          loads_feedback_s_ = monotonic_seconds();
        });
      const std::array<std::string, 4U> control_topics{
        relay_command_publisher_->get_topic_name(), relay_feedback_subscription_->get_topic_name(),
        loads_stop_publisher_->get_topic_name(), loads_feedback_subscription_->get_topic_name()};
      for (std::size_t i = 0U; i < control_topics.size(); ++i) {
        for (std::size_t j = i + 1U; j < control_topics.size(); ++j) {
          if (control_topics[i] == control_topics[j]) {
            throw std::invalid_argument{
                    "automatic discharge control topics collide after ROS remapping: " +
                    control_topics[i]};
          }
        }
      }
    }
    publish_timer_ = create_wall_timer(
      seconds_to_duration(publish_period_s), [this]() {publish();});
    reader_thread_ = std::thread{[this]() {reader_loop();}};
    RCLCPP_INFO(
      get_logger(), "BMS SocketCAN: %s, addresses 0x%02x/0x%02x, automatic control %s",
      can_interface_.c_str(), primary_address_, secondary_address_,
      automatic_discharge_control_ ? "enabled" : "disabled");
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

  std::uint8_t address_parameter(const std::string & name, const std::uint8_t default_value)
  {
    const auto value = declare_parameter<std::int64_t>(name, default_value);
    if (value < 0 || value > std::numeric_limits<std::uint8_t>::max()) {
      throw std::invalid_argument{name + " must be in [0, 255]"};
    }
    return static_cast<std::uint8_t>(value);
  }

  infra_socketcan::Socket connect_can()
  {
    if (!expected_adapter_serial_.empty()) {
      std::ifstream serial_file{
        "/sys/class/net/" + can_interface_ + "/device/../serial"};
      std::string actual_serial;
      if (!std::getline(serial_file, actual_serial) ||
        actual_serial != expected_adapter_serial_)
      {
        RCLCPP_WARN(
          get_logger(), "CAN interface %s does not match configured adapter serial",
          can_interface_.c_str());
        return {};
      }
    }
    std::vector<can_filter> filters(response_ids_.size());
    for (std::size_t i = 0U; i < filters.size(); ++i) {
      filters[i] = make_response_filter(response_ids_[i]);
    }
    infra_socketcan::Socket socket;
    try {
      socket.open(can_interface_, filters);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "CAN connect to %s failed: %s", can_interface_.c_str(), error.what());
      return {};
    }
    RCLCPP_INFO(get_logger(), "connected to %s", can_interface_.c_str());
    return socket;
  }

  void wait_until(const std::chrono::steady_clock::time_point deadline)
  {
    std::unique_lock<std::mutex> lock{wait_mutex_};
    wait_condition_.wait_until(lock, deadline, [this]() {return stop_.load();});
  }

  bool send_discharge_command(
    const infra_socketcan::Socket & socket, const std::uint8_t address, const bool enabled)
  {
    if (!socket.is_open()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "BMS D9 not sent: CAN is disconnected");
      return false;
    }
    can_frame command{};
    command.can_id = CAN_EFF_FLAG |
      make_can_id(kDischargeMosDataId, address, host_address_);
    command.len = 8U;
    command.data[0U] = enabled ? 1U : 0U;
    if (const int result = socket.send(command); result != 0) {
      RCLCPP_ERROR(
        get_logger(), "BMS D9 write failed for 0x%02x: %s", address, std::strerror(-result));
      return false;
    }
    return true;
  }

  void publish_bool(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher,
    const bool value)
  {
    std_msgs::msg::Bool message;
    message.data = value;
    publisher->publish(message);
  }

  void update_discharge_control(const infra_socketcan::Socket & socket)
  {
    PackState primary;
    PackState secondary;
    {
      std::lock_guard<std::mutex> lock{state_mutex_};
      primary = primary_;
      secondary = secondary_;
    }
    std::optional<bool> relay_closed;
    std::optional<double> relay_feedback_s;
    bool loads_stopped = false;
    std::optional<double> loads_feedback_s;
    {
      std::lock_guard<std::mutex> lock{feedback_mutex_};
      relay_closed = relay_closed_;
      relay_feedback_s = relay_feedback_s_;
      loads_stopped = loads_stopped_;
      loads_feedback_s = loads_feedback_s_;
    }
    const DischargeInputs inputs{
      primary, secondary, monotonic_seconds(), loads_stopped, loads_feedback_s,
      relay_closed, relay_feedback_s,
      relay_command_publisher_->get_subscription_count() > 0U &&
      loads_stop_publisher_->get_subscription_count() > 0U};
    const auto previous_phase = controller_->phase();
    const auto action = controller_->update(inputs);
    const auto phase = controller_->phase();
    if (phase != previous_phase) {
      if (phase == DischargePhase::kFaultLatched ||
        phase == DischargePhase::kAwaitingIsolationPolicy)
      {
        RCLCPP_ERROR(get_logger(), "BMS control: %s -> %s; automatic recovery inhibited",
          discharge_phase_name(previous_phase), discharge_phase_name(phase));
      } else if (phase == DischargePhase::kStoppingLoads ||
        phase == DischargePhase::kStoppingSecondary ||
        phase == DischargePhase::kStoppingPrimary || phase == DischargePhase::kOpeningRelay)
      {
        RCLCPP_WARN(get_logger(), "BMS control: %s -> %s",
          discharge_phase_name(previous_phase), discharge_phase_name(phase));
      } else {
        RCLCPP_INFO(get_logger(), "BMS control: %s -> %s",
          discharge_phase_name(previous_phase), discharge_phase_name(phase));
      }
    }
    const bool hold_relay = action != DischargeAction::kOpenRelay &&
      (phase == DischargePhase::kWaitingForRelayClosed ||
      (relay_closed == true &&
      (phase == DischargePhase::kObservingSecondary ||
      phase == DischargePhase::kRunning ||
      phase == DischargePhase::kStoppingLoads ||
      phase == DischargePhase::kStoppingSecondary ||
      phase == DischargePhase::kStoppingPrimary ||
      phase == DischargePhase::kAwaitingIsolationPolicy ||
      (phase == DischargePhase::kFaultLatched && !controller_->secondary_faulted()))));
    if (hold_relay) {
      publish_bool(relay_command_publisher_, true);
    }
    switch (action) {
      case DischargeAction::kNone:
        return;
      case DischargeAction::kStopLoads:
        publish_bool(loads_stop_publisher_, true);
        return;
      case DischargeAction::kEnableSecondary:
        static_cast<void>(send_discharge_command(socket, secondary_address_, true));
        return;
      case DischargeAction::kDisableSecondary:
        static_cast<void>(send_discharge_command(socket, secondary_address_, false));
        return;
      case DischargeAction::kDisablePrimary:
        static_cast<void>(send_discharge_command(socket, primary_address_, false));
        return;
      case DischargeAction::kCloseRelay:
        publish_bool(relay_command_publisher_, true);
        return;
      case DischargeAction::kOpenRelay:
        publish_bool(relay_command_publisher_, false);
        return;
    }
  }

  void reader_loop()
  {
    infra_socketcan::Socket socket;
    std::size_t request_index = 0U;
    auto next_connect = std::chrono::steady_clock::now();
    auto next_request = next_connect;
    auto next_control = next_connect;
    const auto request_spacing = seconds_to_duration(
      request_period_s_ / static_cast<double>(request_ids_.size()));
    auto disconnect = [&]() {
      socket.close();
      next_connect = std::chrono::steady_clock::now() + seconds_to_duration(reconnect_period_s_);
      RCLCPP_WARN(get_logger(), "BMS CAN connection lost; control remains active");
    };
    while (!stop_.load()) {
      const auto now = std::chrono::steady_clock::now();
      // One thread owns the socket. Control has its own deadline, even offline.
      // A D9 write failure does not reset transport or postpone the next tick.
      if (automatic_discharge_control_ && now >= next_control) {
        update_discharge_control(socket);
        next_control = now + seconds_to_duration(request_period_s_);
      }
      if (!socket.is_open() && now >= next_connect) {
        socket = connect_can();
        next_connect = now + seconds_to_duration(reconnect_period_s_);
        request_index = 0U;
        next_request = now;
      }
      if (!socket.is_open()) {
        wait_until(automatic_discharge_control_ ?
          std::min(next_connect, next_control) : next_connect);
        continue;
      }
      if (now >= next_request) {
        const can_frame query = make_query_frame(request_ids_[request_index]);
        if (const int result = socket.send(query); result != 0) {
          RCLCPP_WARN(get_logger(), "BMS query failed: %s", std::strerror(-result));
          disconnect();
          continue;
        }
        request_index = (request_index + 1U) % request_ids_.size();
        next_request = now + request_spacing;
      }
      const auto deadline = automatic_discharge_control_ ?
        std::min(next_request, next_control) : next_request;
      const double remaining_s = std::chrono::duration<double>(
        deadline - std::chrono::steady_clock::now()).count();
      const double wait_s = std::max(0.001, std::min(receive_timeout_s_, remaining_s));
      const int readable = socket.wait_readable(static_cast<int>(std::ceil(wait_s * 1000.0)));
      if (readable == 0 || readable == -EINTR) {
        continue;
      }
      if (readable < 0) {
        disconnect();
        continue;
      }
      can_frame frame{};
      if (socket.receive(frame) != 1) {
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
  }

  void publish_pack(
    const PackState & pack, const std::uint8_t address,
    const rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr & publisher,
    const double timestamp)
  {
    const bool fresh = pack.status_fresh(timestamp, frame_timeout_s_);
    const bool fault_fresh = pack.fault_frame_s &&
      timestamp >= *pack.fault_frame_s &&
      timestamp - *pack.fault_frame_s <= frame_timeout_s_;
    const auto values = project_battery_values(pack.sample, timestamp, frame_timeout_s_);
    const bool present = fresh && std::isfinite(values.voltage_v) &&
      std::isfinite(values.soc_fraction) && !pack.has_fault();
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
    message.temperature = static_cast<float>(nan);
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
    publisher->publish(message);
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
    publish_pack(primary, primary_address_, primary_publisher_, stamp);
    publish_pack(secondary, secondary_address_, secondary_publisher_, stamp);
  }

  std::string can_interface_;
  std::string expected_adapter_serial_;
  bool automatic_discharge_control_{false};
  std::uint8_t primary_address_{kDefaultBmsAddress};
  std::uint8_t secondary_address_{kSecondaryBmsAddress};
  std::uint8_t host_address_{kDefaultHostAddress};
  double request_period_s_{0.2};
  double receive_timeout_s_{0.5};
  double reconnect_period_s_{2.0};
  double frame_timeout_s_{3.0};
  std::array<std::uint32_t, 14U> request_ids_{};
  std::array<std::uint32_t, 16U> response_ids_{};
  std::unique_ptr<DualBatteryController> controller_;
  std::unique_ptr<GoldenPhoenixPack> primary_decoder_;
  std::unique_ptr<GoldenPhoenixPack> secondary_decoder_;
  PackState primary_;
  PackState secondary_;
  std::mutex state_mutex_;
  std::mutex feedback_mutex_;
  std::optional<bool> relay_closed_;
  std::optional<double> relay_feedback_s_;
  bool loads_stopped_{false};
  std::optional<double> loads_feedback_s_;
  std::atomic<bool> stop_{false};
  std::mutex wait_mutex_;
  std::condition_variable wait_condition_;
  std::thread reader_thread_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr primary_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr secondary_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr relay_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr loads_stop_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr relay_feedback_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr loads_feedback_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace

std::shared_ptr<rclcpp::Node> make_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<BmsNode>(options);
}

}  // namespace bms_node
