#ifndef DM_SWERVE_DRIVER__TEST__FAKE_CAN_TRANSPORT_HPP_
#define DM_SWERVE_DRIVER__TEST__FAKE_CAN_TRANSPORT_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>

#include "dm_swerve_driver/can_transport.hpp"
#include "dm_swerve_driver/fake_motor_model.hpp"

namespace dm_swerve_driver::test {

class FakeCanTransport final : public CanTransport {
public:
  explicit FakeCanTransport(MotorLimits limits = MotorLimits{12.5, 30.0, 10.0})
  {
    for (std::uint16_t index{0U}; index < motors_.size(); ++index) {
      motors_[index].emplace(FakeMotorConfig{
          static_cast<std::uint16_t>(index + 1U),
          static_cast<std::uint16_t>(0x11U + index),
          limits,
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

  void write_batch(const std::vector<CanFrame> & frames) override
  {
    if (!open_) {
      throw SocketCanError{"fake transport is closed", EBADF};
    }
    batches_.push_back(frames);
    for (const auto & frame : frames) {
      if (frame.id == kRegisterCanId && !register_replies_enabled_) {
        continue;
      }
      for (auto & motor : motors_) {
        auto response = motor->handle_frame(frame, std::chrono::milliseconds{10});
        if (!response.has_value()) {
          continue;
        }
        if (next_error_override_[response->id & 0x0FU].has_value()) {
          response->data[0] = static_cast<std::uint8_t>(
            (static_cast<std::uint8_t>(*next_error_override_[response->id & 0x0FU]) << 4U) |
            (response->data[0] & 0x0FU));
          next_error_override_[response->id & 0x0FU].reset();
        }
        if (!drop_all_feedback_ &&
          (!dropped_mst_id_.has_value() || response->id != *dropped_mst_id_))
        {
          pending_.push_back(ReceivedCanFrame{*response, {}});
        }
        break;
      }
    }
  }

  [[nodiscard]] std::vector<ReceivedCanFrame> collect(
    std::size_t expected_count,
    std::chrono::steady_clock::time_point) override
  {
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
  void set_dropped_mst_id(std::optional<std::uint16_t> mst_id) noexcept
  {
    dropped_mst_id_ = mst_id;
  }
  void set_fail_open(bool fail) noexcept {fail_open_ = fail;}
  void set_drop_all_feedback(bool drop) noexcept {drop_all_feedback_ = drop;}
  void override_next_error(std::uint16_t mst_id, MotorError error)
  {
    next_error_override_[mst_id & 0x0FU] = error;
  }
  void clear_batches() {batches_.clear();}

  [[nodiscard]] const std::vector<std::vector<CanFrame>> & batches() const noexcept
  {
    return batches_;
  }

private:
  std::array<std::optional<FakeMotorModel>, kMotorCount> motors_{};
  std::deque<ReceivedCanFrame> pending_;
  std::vector<std::vector<CanFrame>> batches_;
  std::optional<std::uint16_t> dropped_mst_id_;
  std::array<std::optional<MotorError>, 16U> next_error_override_{};
  bool open_{false};
  bool fail_open_{false};
  bool drop_all_feedback_{false};
  bool register_replies_enabled_{true};
};

}  // namespace dm_swerve_driver::test

#endif  // DM_SWERVE_DRIVER__TEST__FAKE_CAN_TRANSPORT_HPP_
