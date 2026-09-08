#ifndef DM_SWERVE_DRIVER__TEST__FAKE_CAN_TRANSPORT_HPP_
#define DM_SWERVE_DRIVER__TEST__FAKE_CAN_TRANSPORT_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>

#include "dm_swerve_driver/can_transport.hpp"
#include "dm_swerve_driver/fake_motor_model.hpp"
#include "dm_swerve_driver/feedback_router.hpp"

namespace dm_swerve_driver::test {

class FakeCanTransport final : public CanTransport {
public:
  explicit FakeCanTransport(MotorLimits limits = MotorLimits{12.5, 30.0, 10.0})
  : FakeCanTransport(uniform_limits(limits))
  {
  }

  explicit FakeCanTransport(const std::array<MotorLimits, kMotorCount> & limits)
  : limits_{limits}
  {
    for (std::uint16_t index{0U}; index < motors_.size(); ++index) {
      motors_[index].emplace(FakeMotorConfig{
          static_cast<std::uint16_t>(index + 1U),
          static_cast<std::uint16_t>(0x11U + index),
          limits[index],
          std::chrono::milliseconds{30}});
    }
  }

  void open() override
  {
    if (fail_open_) {
      throw SocketCanError{"injected open failure", EIO};
    }
    open_ = true;
  }

  void close() noexcept override {open_ = false;}
  [[nodiscard]] bool is_open() const noexcept override {return open_;}
  [[nodiscard]] bool allows_fallback_limits() const noexcept override {return true;}

  void write_batch(const std::vector<CanFrame> & frames) override
  {
    if (!open_) {
      throw SocketCanError{"fake transport is closed", EBADF};
    }
    if (fail_io_) {
      throw SocketCanError{"injected transport write failure", EIO};
    }
    batches_.push_back(frames);
    for (const auto & frame : frames) {
      std::optional<RegisterReply> register_request;
      if (frame.id == kRegisterCanId) {
        register_request = decode_register_reply(frame);
        if (!register_replies_enabled_ ||
          (suppressed_register_.has_value() &&
          register_request->register_id == static_cast<std::uint8_t>(*suppressed_register_)))
        {
          continue;
        }
      }
      const auto special = decode_special_command(frame);
      if (special == SpecialCommand::enable &&
        ignored_enable_esc_id_.has_value() && frame.id == *ignored_enable_esc_id_)
      {
        continue;
      }
      for (std::size_t motor_index{0U}; motor_index < motors_.size(); ++motor_index) {
        auto & motor = motors_[motor_index];
        auto response = motor->handle_frame(frame, std::chrono::milliseconds{10});
        if (!response.has_value()) {
          continue;
        }
        const bool regular_mit_command = frame.id != kRegisterCanId &&
          !special.has_value() && motor->enabled();
        if (regular_mit_command &&
          next_error_override_[response->id & 0x0FU].has_value())
        {
          response->data[0] = static_cast<std::uint8_t>(
            (static_cast<std::uint8_t>(*next_error_override_[response->id & 0x0FU]) << 4U) |
            (response->data[0] & 0x0FU));
          next_error_override_[response->id & 0x0FU].reset();
        }
        if (register_request.has_value() &&
          register_request->register_id ==
          static_cast<std::uint8_t>(RegisterId::multi_turn_position) &&
          multi_turn_override_[register_request->motor_can_id & 0x0FU].has_value())
        {
          const float value =
            *multi_turn_override_[register_request->motor_can_id & 0x0FU];
          std::uint32_t raw_value{0U};
          std::memcpy(&raw_value, &value, sizeof(raw_value));
          for (std::size_t byte{0U}; byte < 4U; ++byte) {
            response->data[4U + byte] = static_cast<std::uint8_t>(
              (raw_value >> (8U * byte)) & 0xFFU);
          }
        }
        if (frame.id != kRegisterCanId &&
          position_override_[response->id & 0x0FU].has_value())
        {
          auto feedback = decode_motor_feedback(*response, limits_[motor_index]);
          feedback.position = *position_override_[response->id & 0x0FU];
          *response = encode_motor_feedback(response->id, feedback, limits_[motor_index]);
        }
        if (!drop_all_feedback_ &&
          (!dropped_mst_id_.has_value() || response->id != *dropped_mst_id_))
        {
          pending_.push_back(ReceivedCanFrame{
              *response,
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())});
        }
        break;
      }
    }
  }

  [[nodiscard]] std::vector<ReceivedCanFrame> collect(
    std::size_t expected_count,
    std::chrono::steady_clock::time_point) override
  {
    if (fail_collect_) {
      throw SocketCanError{"injected transport collect failure", EIO};
    }
    std::vector<ReceivedCanFrame> result;
    while (!pending_.empty() && result.size() < expected_count) {
      result.push_back(pending_.front());
      pending_.pop_front();
    }
    return result;
  }

  void inject(ReceivedCanFrame frame) {pending_.push_back(std::move(frame));}
  void set_register_replies_enabled(bool enabled) noexcept
  {
    register_replies_enabled_ = enabled;
  }
  void set_suppressed_register(std::optional<RegisterId> register_id) noexcept
  {
    suppressed_register_ = register_id;
  }
  void set_multi_turn_override(std::uint16_t esc_id, std::optional<float> position) noexcept
  {
    multi_turn_override_[esc_id & 0x0FU] = position;
  }
  void set_ignored_enable_esc_id(std::optional<std::uint16_t> esc_id) noexcept
  {
    ignored_enable_esc_id_ = esc_id;
  }
  void set_dropped_mst_id(std::optional<std::uint16_t> mst_id) noexcept
  {
    dropped_mst_id_ = mst_id;
  }
  void set_fail_open(bool fail) noexcept {fail_open_ = fail;}
  void set_fail_io(bool fail) noexcept {fail_io_ = fail;}
  void set_fail_collect(bool fail) noexcept {fail_collect_ = fail;}
  void force_close() noexcept {open_ = false;}
  void set_drop_all_feedback(bool drop) noexcept {drop_all_feedback_ = drop;}
  void override_next_error(std::uint16_t mst_id, MotorError error)
  {
    next_error_override_[mst_id & 0x0FU] = error;
  }
  void set_position_override(std::uint16_t mst_id, std::optional<double> position)
  {
    position_override_[mst_id & 0x0FU] = position;
  }
  void clear_batches() {batches_.clear();}

  [[nodiscard]] const std::vector<std::vector<CanFrame>> & batches() const noexcept
  {
    return batches_;
  }

private:
  [[nodiscard]] static std::array<MotorLimits, kMotorCount> uniform_limits(
    const MotorLimits & limits)
  {
    std::array<MotorLimits, kMotorCount> result{};
    result.fill(limits);
    return result;
  }

  std::array<std::optional<FakeMotorModel>, kMotorCount> motors_{};
  std::array<MotorLimits, kMotorCount> limits_{};
  std::deque<ReceivedCanFrame> pending_;
  std::vector<std::vector<CanFrame>> batches_;
  std::optional<std::uint16_t> dropped_mst_id_;
  std::optional<std::uint16_t> ignored_enable_esc_id_;
  std::optional<RegisterId> suppressed_register_;
  std::array<std::optional<MotorError>, 16U> next_error_override_{};
  std::array<std::optional<double>, 16U> position_override_{};
  std::array<std::optional<float>, 16U> multi_turn_override_{};
  bool open_{false};
  bool fail_open_{false};
  bool fail_io_{false};
  bool fail_collect_{false};
  bool drop_all_feedback_{false};
  bool register_replies_enabled_{true};
};

}  // namespace dm_swerve_driver::test

#endif  // DM_SWERVE_DRIVER__TEST__FAKE_CAN_TRANSPORT_HPP_
