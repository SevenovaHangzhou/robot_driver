#include "lpms_nav3_can/decoder.hpp"
#include "lpms_nav3_can/ros_conversion.hpp"
#include "lpms_nav3_can/socket_can.hpp"
#include "lpms_nav3_can/node.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include "robot_interfaces_qos/profiles.hpp"
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace lpms_nav3_can
{
namespace
{

using SteadyClock = std::chrono::steady_clock;

class SocketHandle
{
public:
  SocketHandle() = default;
  explicit SocketHandle(const int fd) noexcept
  : fd_(fd)
  {}

  ~SocketHandle()
  {
    reset();
  }

  SocketHandle(const SocketHandle &) = delete;
  SocketHandle & operator=(const SocketHandle &) = delete;

  SocketHandle(SocketHandle && other) noexcept
  : fd_(std::exchange(other.fd_, -1))
  {}

  SocketHandle & operator=(SocketHandle && other) noexcept
  {
    if (this != &other) {
      reset();
      fd_ = std::exchange(other.fd_, -1);
    }
    return *this;
  }

  void reset(const int fd = -1) noexcept
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    fd_ = fd;
  }

  [[nodiscard]] int get() const noexcept {return fd_;}
  [[nodiscard]] bool valid() const noexcept {return fd_ >= 0;}

private:
  int fd_{-1};
};

[[nodiscard]] std::chrono::milliseconds positive_duration(
  const std::int64_t value, const char * const parameter_name)
{
  if (value <= 0) {
    throw std::invalid_argument{std::string{parameter_name} + " must be positive"};
  }
  return std::chrono::milliseconds{value};
}

[[nodiscard]] diagnostic_msgs::msg::KeyValue diagnostic_value(
  std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = std::move(key);
  item.value = std::move(value);
  return item;
}

[[nodiscard]] const char * heartbeat_state_name(const std::uint8_t state) noexcept
{
  switch (state) {
    case 0x00U:
      return "boot-up";
    case 0x04U:
      return "stopped";
    case 0x05U:
      return "operational";
    case 0x7fU:
      return "pre-operational";
    default:
      return "unknown";
  }
}

}  // namespace

class LpmsNav3CanNode final : public rclcpp::Node
{
public:
  explicit LpmsNav3CanNode(const rclcpp::NodeOptions & options)
  : Node("lpms_nav3_can_node", options),
    node_id_(read_node_id()),
    decoder_(node_id_)
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    convert_to_ros_ = declare_parameter<bool>("convert_to_ros_convention", false);
    require_heartbeat_ = declare_parameter<bool>("require_heartbeat", true);
    poll_period_ = positive_duration(
      declare_parameter<std::int64_t>("poll_period_ms", 2), "poll_period_ms");
    reconnect_period_ = positive_duration(
      declare_parameter<std::int64_t>("reconnect_period_ms", 1000),
      "reconnect_period_ms");
    pdo_timeout_ = positive_duration(
      declare_parameter<std::int64_t>("pdo_timeout_ms", 250), "pdo_timeout_ms");
    assembly_timeout_ = positive_duration(
      declare_parameter<std::int64_t>("assembly_timeout_ms", 50),
      "assembly_timeout_ms");
    heartbeat_timeout_ = positive_duration(
      declare_parameter<std::int64_t>("heartbeat_timeout_ms", 2500),
      "heartbeat_timeout_ms");

    if (can_interface_.empty() || can_interface_ == "TBD" ||
      can_interface_.size() >= IFNAMSIZ)
    {
      throw std::invalid_argument{"can_interface must be explicitly configured"};
    }
    if (is_reserved_robot_interface(can_interface_)) {
      throw std::invalid_argument{
              "refusing reserved interface " + can_interface_ +
              "; shared Gen3 CAN admission is not implemented"};
    }
    if (frame_id_.empty() || frame_id_ == "TBD" || frame_id_.front() == '/' ||
      frame_id_.find_first_of(" \t\r\n") != std::string::npos)
    {
      throw std::invalid_argument{"frame_id must be explicitly configured without a leading slash"};
    }

    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "~/data", robot_interfaces_qos::fast_state());
    magnetic_field_publisher_ = create_publisher<sensor_msgs::msg::MagneticField>(
      "~/mag", robot_interfaces_qos::fast_state());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "~/diagnostics", robot_interfaces_qos::diagnostic());

    next_connect_attempt_ = SteadyClock::time_point::min();
    poll_timer_ = create_wall_timer(poll_period_, [this]() {poll_bus();});
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds{1}, [this]() {publish_diagnostics();});

    RCLCPP_INFO(
      get_logger(),
      "LPMS-NAV3 CAN driver configured: interface=%s node_id=%u frame_id=%s",
      can_interface_.c_str(), static_cast<unsigned int>(node_id_), frame_id_.c_str());
    try_connect();
  }

private:
  [[nodiscard]] std::uint8_t read_node_id()
  {
    const auto value = declare_parameter<std::int64_t>("node_id", 0);
    if (value < 1 || value > 127) {
      throw std::invalid_argument{"node_id must be in [1, 127]"};
    }
    return static_cast<std::uint8_t>(value);
  }

  void record_socket_error(std::string message)
  {
    if (message != last_socket_error_) {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
      last_socket_error_ = std::move(message);
    }
  }

  void try_connect()
  {
    const auto current_time = SteadyClock::now();
    if (socket_.valid() || current_time < next_connect_attempt_) {
      return;
    }
    next_connect_attempt_ = current_time + reconnect_period_;
    ++connect_attempt_count_;

    const unsigned int interface_index = if_nametoindex(can_interface_.c_str());
    if (interface_index == 0U) {
      record_socket_error(
        "SocketCAN interface " + can_interface_ + " is unavailable: " +
        std::strerror(errno));
      return;
    }

    SocketHandle candidate{
      ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW)};
    if (!candidate.valid()) {
      record_socket_error("failed to create CAN_RAW socket: " + std::string{std::strerror(errno)});
      return;
    }

    const auto filters = make_can_filters(node_id_);
    if (::setsockopt(
        candidate.get(), SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
        static_cast<socklen_t>(sizeof(filters))) != 0)
    {
      record_socket_error("failed to configure CAN filters: " + std::string{std::strerror(errno)});
      return;
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(interface_index);
    if (::bind(
        candidate.get(), reinterpret_cast<const sockaddr *>(&address),
        static_cast<socklen_t>(sizeof(address))) != 0)
    {
      record_socket_error(
        "failed to bind " + can_interface_ + ": " + std::strerror(errno));
      return;
    }

    socket_ = std::move(candidate);
    bound_interface_index_ = interface_index;
    decoder_.reset();
    has_partial_assembly_ = false;
    has_sample_ = false;
    has_heartbeat_ = false;
    heartbeat_state_ = 0U;
    last_socket_error_.clear();
    RCLCPP_INFO(get_logger(), "connected to SocketCAN interface %s", can_interface_.c_str());

  }

  void disconnect(std::string reason)
  {
    socket_.reset();
    bound_interface_index_ = 0U;
    decoder_.reset();
    has_partial_assembly_ = false;
    has_sample_ = false;
    has_heartbeat_ = false;
    heartbeat_state_ = 0U;
    record_socket_error(std::move(reason));
    next_connect_attempt_ = SteadyClock::now() + reconnect_period_;
  }

  void poll_bus()
  {
    if (!socket_.valid()) {
      try_connect();
      return;
    }
    const unsigned int current_interface_index = if_nametoindex(can_interface_.c_str());
    if (current_interface_index == 0U || current_interface_index != bound_interface_index_) {
      disconnect("SocketCAN interface disappeared or was replaced: " + can_interface_);
      return;
    }

    for (std::size_t iteration = 0U; iteration < 256U; ++iteration) {
      can_frame raw_frame{};
      const auto bytes_read = ::recv(socket_.get(), &raw_frame, sizeof(raw_frame), MSG_DONTWAIT);
      if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return;
        }
        disconnect("SocketCAN receive failed: " + std::string{std::strerror(errno)});
        return;
      }
      if (bytes_read != static_cast<ssize_t>(sizeof(raw_frame))) {
        ++malformed_frame_count_;
        continue;
      }

      CanFrame frame;
      frame.is_extended = (raw_frame.can_id & CAN_EFF_FLAG) != 0U;
      frame.is_remote = (raw_frame.can_id & CAN_RTR_FLAG) != 0U;
      frame.is_error = (raw_frame.can_id & CAN_ERR_FLAG) != 0U;
      frame.id = raw_frame.can_id & (frame.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
      frame.dlc = raw_frame.can_dlc;
      std::copy(std::begin(raw_frame.data), std::end(raw_frame.data), frame.data.begin());

      const auto receive_time = SteadyClock::now();
      const auto node = static_cast<std::uint32_t>(node_id_);
      const bool is_pdo =
        frame.id == 0x180U + node || frame.id == 0x280U + node ||
        frame.id == 0x380U + node || frame.id == 0x480U + node;
      if (
        is_pdo && has_partial_assembly_ &&
        receive_time - assembly_started_time_ > assembly_timeout_)
      {
        decoder_.reset();
        has_partial_assembly_ = false;
        ++dropped_assembly_count_;
      }

      const bool starts_new_assembly = is_pdo && !has_partial_assembly_;
      const auto result = decoder_.consume(frame);
      if (!result.recognized) {
        continue;
      }
      ++recognized_frame_count_;
      if (result.malformed) {
        ++malformed_frame_count_;
        continue;
      }
      if (is_pdo && starts_new_assembly) {
        assembly_started_time_ = receive_time;
        has_partial_assembly_ = true;
      }
      if (result.heartbeat_state.has_value()) {
        heartbeat_state_ = *result.heartbeat_state;
        last_heartbeat_time_ = receive_time;
        has_heartbeat_ = true;
      }
      if (result.sample.has_value()) {
        has_partial_assembly_ = false;
        last_sample_time_ = receive_time;
        has_sample_ = true;
        ++published_sample_count_;
        publish_sample(*result.sample);
      }
    }
  }

  void publish_sample(const ImuSample & raw_sample)
  {
    const auto sample = convert_to_ros_convention(raw_sample, convert_to_ros_);
    auto imu_message = make_imu_message(sample);
    auto magnetic_field_message = make_magnetic_field_message(sample);
    const auto stamp = now();
    imu_message.header.stamp = stamp;
    imu_message.header.frame_id = frame_id_;
    magnetic_field_message.header = imu_message.header;
    imu_publisher_->publish(imu_message);
    magnetic_field_publisher_->publish(magnetic_field_message);
  }

  [[nodiscard]] static double age_ms(
    const SteadyClock::time_point timestamp, const SteadyClock::time_point now) noexcept
  {
    return std::chrono::duration<double, std::milli>{now - timestamp}.count();
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name() + std::string{": LPMS-NAV3 CAN"};
    status.hardware_id =
      "lpms-nav3-node-" + std::to_string(node_id_) + "@" + can_interface_;

    const auto current_time = SteadyClock::now();
    const double sample_age = has_sample_ ? age_ms(last_sample_time_, current_time) : -1.0;
    const double heartbeat_age =
      has_heartbeat_ ? age_ms(last_heartbeat_time_, current_time) : -1.0;
    if (!socket_.valid()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "SocketCAN interface unavailable";
    } else if (!has_sample_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for complete TPDO set";
    } else if (sample_age > static_cast<double>(pdo_timeout_.count())) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "TPDO data stale";
    } else if (
      require_heartbeat_ &&
      (!has_heartbeat_ || heartbeat_age > static_cast<double>(heartbeat_timeout_.count())))
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "CANopen heartbeat missing or stale";
    } else if (require_heartbeat_ && heartbeat_state_ != 0x05U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "CANopen node is not operational";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "LPMS-NAV3 CAN data healthy";
    }

    status.values.push_back(diagnostic_value("can_interface", can_interface_));
    status.values.push_back(diagnostic_value("node_id", std::to_string(node_id_)));
    status.values.push_back(diagnostic_value("socket_connected", socket_.valid() ? "true" : "false"));
    status.values.push_back(diagnostic_value("sample_age_ms", std::to_string(sample_age)));
    status.values.push_back(diagnostic_value("heartbeat_age_ms", std::to_string(heartbeat_age)));
    status.values.push_back(
      diagnostic_value("heartbeat_state", heartbeat_state_name(heartbeat_state_)));
    status.values.push_back(
      diagnostic_value("published_samples", std::to_string(published_sample_count_)));
    status.values.push_back(
      diagnostic_value("recognized_frames", std::to_string(recognized_frame_count_)));
    status.values.push_back(
      diagnostic_value("malformed_frames", std::to_string(malformed_frame_count_)));
    status.values.push_back(
      diagnostic_value("dropped_assemblies", std::to_string(dropped_assembly_count_)));
    status.values.push_back(
      diagnostic_value("connect_attempts", std::to_string(connect_attempt_count_)));
    status.values.push_back(diagnostic_value("last_socket_error", last_socket_error_));
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  std::string can_interface_;
  std::string frame_id_;
  const std::uint8_t node_id_;
  Decoder decoder_;
  bool convert_to_ros_{false};
  bool require_heartbeat_{true};
  std::chrono::milliseconds poll_period_{2};
  std::chrono::milliseconds reconnect_period_{1000};
  std::chrono::milliseconds pdo_timeout_{250};
  std::chrono::milliseconds assembly_timeout_{50};
  std::chrono::milliseconds heartbeat_timeout_{2500};
  SocketHandle socket_;
  unsigned int bound_interface_index_{0U};
  SteadyClock::time_point next_connect_attempt_{};
  SteadyClock::time_point last_sample_time_{};
  SteadyClock::time_point last_heartbeat_time_{};
  SteadyClock::time_point assembly_started_time_{};
  bool has_sample_{false};
  bool has_heartbeat_{false};
  bool has_partial_assembly_{false};
  std::uint8_t heartbeat_state_{0U};
  std::uint64_t published_sample_count_{0U};
  std::uint64_t recognized_frame_count_{0U};
  std::uint64_t malformed_frame_count_{0U};
  std::uint64_t dropped_assembly_count_{0U};
  std::uint64_t connect_attempt_count_{0U};
  std::string last_socket_error_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

rclcpp::Node::SharedPtr make_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<LpmsNav3CanNode>(options);
}

}  // namespace lpms_nav3_can
