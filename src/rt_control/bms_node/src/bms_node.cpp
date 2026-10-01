#include "bms_node/node.hpp"

#include "bms_node/can_transport.hpp"
#include "bms_node/protocol.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
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
    can_interface_ = declare_parameter<std::string>("can_interface", "can1");
    const auto protocol = declare_parameter<std::string>(
      "protocol", "golden_phoenix_v1_1");
    const auto can_bitrate = declare_parameter<std::int64_t>("can_bitrate", 250000);
    if (protocol != "golden_phoenix_v1_1" || can_bitrate != 250000) {
      throw std::invalid_argument{
              "bms_node supports only golden_phoenix_v1_1 at 250000 bit/s"};
    }
    receive_timeout_s_ = positive_parameter("receive_timeout_s", 0.5);
    reconnect_period_s_ = positive_parameter("reconnect_period_s", 2.0);
    const double publish_period_s = positive_parameter("publish_period_s", 5.0);
    frame_timeout_s_ = positive_parameter("frame_timeout_s", 3.0);
    request_period_s_ = positive_parameter("request_period_s", 0.2);
    const auto bms_address = address_parameter("bms_address", kDefaultBmsAddress);
    const auto host_address = address_parameter("host_address", kDefaultHostAddress);
    const auto order = parse_byte_order(
      declare_parameter<std::string>("multi_byte_order", "auto"));
    const auto topic = declare_parameter<std::string>(
      "battery_state_topic", "/battery_state");

    if (can_interface_.empty() || can_interface_.size() >= IFNAMSIZ) {
      throw std::invalid_argument{"can_interface must name a valid CAN network interface"};
    }
    if (topic.empty()) {
      throw std::invalid_argument{"battery_state_topic must not be empty"};
    }

    request_id_ = make_can_id(kTotalStatusDataId, bms_address, host_address);
    response_id_ = make_can_id(kTotalStatusDataId, host_address, bms_address);
    decoder_ = std::make_unique<GoldenPhoenixDecoder>(order, bms_address, host_address);
    publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
      topic, robot_interfaces_qos::state());
    publish_timer_ = create_wall_timer(
      seconds_to_duration(publish_period_s), [this]() {publish();});
    reader_thread_ = std::thread{[this]() {reader_loop();}};

    RCLCPP_INFO(
      get_logger(),
      "Golden Phoenix BMS ready: interface=%s bitrate=250000 request=0x%08x response=0x%08x",
      can_interface_.c_str(), request_id_, response_id_);
  }

  ~BmsNode() override
  {
    stop_.store(true);
    wait_condition_.notify_all();
    close_socket();
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
  }

private:
  double positive_parameter(const std::string & name, const double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument{name + " must be finite and greater than zero"};
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

  int connect_can()
  {
    const unsigned int interface_index = if_nametoindex(can_interface_.c_str());
    if (interface_index == 0U) {
      RCLCPP_WARN(
        get_logger(), "BMS CAN interface %s is unavailable", can_interface_.c_str());
      return -1;
    }

    const int descriptor = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (descriptor < 0) {
      RCLCPP_WARN(get_logger(), "BMS CAN open failed: %s", std::strerror(errno));
      return -1;
    }

    const can_filter filter = make_response_filter(response_id_);
    if (::setsockopt(
        descriptor, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) != 0)
    {
      RCLCPP_WARN(get_logger(), "BMS CAN filter failed: %s", std::strerror(errno));
      ::close(descriptor);
      return -1;
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(interface_index);
    if (::bind(
        descriptor, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0)
    {
      RCLCPP_WARN(
        get_logger(), "BMS CAN bind to %s failed: %s",
        can_interface_.c_str(), std::strerror(errno));
      ::close(descriptor);
      return -1;
    }

    {
      std::lock_guard<std::mutex> lock{socket_mutex_};
      if (stop_.load()) {
        ::close(descriptor);
        return -1;
      }
      socket_fd_ = descriptor;
    }
    RCLCPP_INFO(get_logger(), "connected to BMS CAN interface %s", can_interface_.c_str());
    return descriptor;
  }

  void close_socket()
  {
    std::lock_guard<std::mutex> lock{socket_mutex_};
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  void disconnect(const int descriptor)
  {
    std::lock_guard<std::mutex> lock{socket_mutex_};
    if (socket_fd_ == descriptor) {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  void wait_for_reconnect()
  {
    std::unique_lock<std::mutex> lock{wait_mutex_};
    wait_condition_.wait_for(
      lock, seconds_to_duration(reconnect_period_s_), [this]() {return stop_.load();});
  }

  bool send_query(const int descriptor)
  {
    const can_frame request = make_query_frame(request_id_);
    const ssize_t written = ::write(descriptor, &request, sizeof(request));
    if (written == static_cast<ssize_t>(sizeof(request))) {
      return true;
    }
    if (!stop_.load()) {
      RCLCPP_WARN(get_logger(), "BMS CAN query failed: %s", std::strerror(errno));
    }
    return false;
  }

  void reader_loop()
  {
    while (!stop_.load()) {
      const int descriptor = connect_can();
      if (descriptor < 0) {
        wait_for_reconnect();
        continue;
      }

      auto next_query = std::chrono::steady_clock::now();
      bool connected = true;
      while (connected && !stop_.load()) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_query) {
          connected = send_query(descriptor);
          next_query = now + seconds_to_duration(request_period_s_);
          if (!connected) {
            break;
          }
        }

        const auto remaining = std::chrono::duration<double>(
          next_query - std::chrono::steady_clock::now()).count();
        const double wait_s = std::max(0.001, std::min(receive_timeout_s_, remaining));
        const int wait_ms = static_cast<int>(std::ceil(wait_s * 1000.0));
        pollfd event{descriptor, POLLIN, 0};
        const int result = ::poll(&event, 1, wait_ms);
        if (result == 0 || (result < 0 && errno == EINTR)) {
          continue;
        }
        if (result < 0 || (event.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
          connected = false;
          continue;
        }
        if ((event.revents & POLLIN) == 0) {
          continue;
        }

        can_frame frame{};
        const ssize_t received = ::read(descriptor, &frame, sizeof(frame));
        if (received != static_cast<ssize_t>(sizeof(frame))) {
          connected = false;
          continue;
        }
        const std::uint8_t * data = nullptr;
        std::size_t size = 0U;
        if (!decode_response_frame(frame, response_id_, data, size)) {
          continue;
        }
        std::lock_guard<std::mutex> lock{sample_mutex_};
        static_cast<void>(decoder_->ingest(
          sample_, response_id_, data, size, monotonic_seconds()));
      }

      disconnect(descriptor);
      if (!stop_.load()) {
        RCLCPP_WARN(get_logger(), "BMS CAN connection lost; reconnecting");
        wait_for_reconnect();
      }
    }
  }

  void publish()
  {
    BmsSample sample;
    {
      std::lock_guard<std::mutex> lock{sample_mutex_};
      sample = sample_;
    }
    const BatteryValues values = project_battery_values(
      sample, monotonic_seconds(), frame_timeout_s_);
    const bool fresh = std::isfinite(values.voltage_v) && std::isfinite(values.soc_fraction);
    const double nan = std::numeric_limits<double>::quiet_NaN();

    sensor_msgs::msg::BatteryState message;
    message.header.stamp = now();
    message.header.frame_id = can_interface_;
    message.voltage = static_cast<float>(values.voltage_v);
    message.temperature = static_cast<float>(nan);
    message.current = static_cast<float>(nan);
    message.charge = static_cast<float>(nan);
    message.capacity = static_cast<float>(nan);
    message.design_capacity = static_cast<float>(nan);
    message.percentage = static_cast<float>(values.soc_fraction);
    message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    message.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    message.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    message.present = fresh;
    publisher_->publish(message);
  }

  std::string can_interface_;
  double receive_timeout_s_{0.5};
  double reconnect_period_s_{2.0};
  double frame_timeout_s_{3.0};
  double request_period_s_{0.2};
  std::uint32_t request_id_{0U};
  std::uint32_t response_id_{0U};
  std::unique_ptr<GoldenPhoenixDecoder> decoder_;
  BmsSample sample_;
  std::mutex sample_mutex_;
  std::mutex socket_mutex_;
  int socket_fd_{-1};
  std::atomic<bool> stop_{false};
  std::mutex wait_mutex_;
  std::condition_variable wait_condition_;
  std::thread reader_thread_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace

std::shared_ptr<rclcpp::Node> make_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<BmsNode>(options);
}

}  // namespace bms_node
