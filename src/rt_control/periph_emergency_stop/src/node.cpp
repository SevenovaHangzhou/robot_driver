#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "infra_modbus/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rt_control_interfaces/msg/emergency_stop_state.hpp"

namespace periph_emergency_stop
{
using State = rt_control_interfaces::msg::EmergencyStopState;
using Clock = std::chrono::steady_clock;

class EmergencyStopNode final : public rclcpp::Node
{
public:
  EmergencyStopNode() : Node("emergency_stop")
  {
    // Hardware mapping and timing have no guessed deployment defaults.
    const auto host = declare_parameter<std::string>("host", "");
    const auto interface = declare_parameter<std::string>("interface", "");
    const auto port = integer("port", 1, 65535);
    unit_ = static_cast<uint8_t>(integer("unit_id", 0, 255));
    address_ = static_cast<uint16_t>(integer("input_address", 0, 65535));
    pressed_level_ = integer("pressed_level", 0, 1) != 0;
    const auto poll_ms = integer("poll_period_ms", 1, 2147483647);
    const auto timeout_ms = integer("request_timeout_ms", 1, 2147483647);
    stale_after_ = std::chrono::milliseconds(integer("stale_timeout_ms", 1, 2147483647));
    client_ = std::make_unique<infra_modbus::Client>(
      host, static_cast<uint16_t>(port), std::chrono::milliseconds(timeout_ms), interface);
    std::random_device random;
    for (auto & byte : snapshot_.producer_instance_id.uuid) {
      byte = static_cast<uint8_t>(random());
    }
    snapshot_.detail = "awaiting_first_sample";
    snapshot_.sample_age_seconds = -1.0;
    publisher_ = create_publisher<State>(
      "/plc/emergency_stop/state", rclcpp::QoS(1).reliable().transient_local());
    publish();
    // Separate non-RT callback groups keep UNKNOWN/stale broadcasts running during TCP waits.
    poll_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publish_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(poll_ms), [this]() { poll(); }, poll_group_);
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(poll_ms), [this]() { publish(); }, publish_group_);
  }

private:
  int64_t integer(const std::string & name, int64_t minimum, int64_t maximum)
  {
    const auto value = declare_parameter<int64_t>(name, -1);
    if (value < minimum || value > maximum) {
      throw std::invalid_argument(
        name + " must be explicitly configured in [" + std::to_string(minimum) + ", " +
        std::to_string(maximum) + "]");
    }
    return value;
  }

  void poll()
  {
    try {
      const auto reply = client_->transact(
        unit_, {0x02, static_cast<uint8_t>(address_ >> 8), static_cast<uint8_t>(address_), 0, 1});
      // A one-bit FC02 reply has exactly one data byte; unused bits must be zero.
      if (reply.size() != 3 || reply[0] != 0x02 || reply[1] != 1 || reply[2] > 1) {
        throw std::runtime_error("invalid_one_bit_FC02_response");
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.raw_level = reply[2] != 0;
        snapshot_.valid = true;
        snapshot_.sample_stamp = now();
        ++snapshot_.sample_sequence;
        last_sample_ = Clock::now();
        snapshot_.detail = "";
      }
    } catch (const std::exception & error) {
      client_->close();
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot_.valid = false;
      snapshot_.detail = error.what();
    }
    publish();
  }

  void publish()
  {
    // Hold through publish so a delayed heartbeat cannot overtake a newer pressed/error sample.
    std::lock_guard<std::mutex> lock(mutex_);
    auto message = snapshot_;
    message.header.stamp = now();
    if (message.sample_sequence != 0) {
      const auto age = Clock::now() - last_sample_;
      message.sample_age_seconds = std::chrono::duration<double>(age).count();
      if (message.valid && age >= stale_after_) {
        message.valid = false;
        message.detail = "sample_stale";
      }
    }
    message.state = !message.valid
                      ? State::UNKNOWN
                      : (message.raw_level == pressed_level_ ? State::PRESSED : State::RELEASED);
    message.stop_requested = message.state != State::RELEASED;
    publisher_->publish(message);
  }

  std::mutex mutex_;
  State snapshot_;
  Clock::time_point last_sample_{};
  std::chrono::milliseconds stale_after_{0};
  uint8_t unit_{0};
  uint16_t address_{0};
  bool pressed_level_{false};
  std::unique_ptr<infra_modbus::Client> client_;
  rclcpp::Publisher<State>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr poll_group_, publish_group_;
  rclcpp::TimerBase::SharedPtr poll_timer_, publish_timer_;
};
}  // namespace periph_emergency_stop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<periph_emergency_stop::EmergencyStopNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("emergency_stop"), "%s", error.what());
      result = 1;
    }
  }
  rclcpp::shutdown();
  return result;
}
