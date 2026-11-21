#ifndef HW_ETHERCAT__PP_AXIS_HPP_
#define HW_ETHERCAT__PP_AXIS_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "hw_semantic_components/cia402_state.hpp"

namespace hw_ethercat
{
// Values are internal ros2_control state, not a public ROS message contract.
enum class PpState : int {
  fault = -1,
  unavailable = 0,
  idle = 1,
  preparing = 2,
  waiting_ack = 3,
  releasing_ack = 4,
  moving = 5,
  reached = 6,
  halting = 7,
  halted = 8
};

// One CiA-402 Profile Position axis in calibrated units. Device adapters add their own
// per-target objects (force limit, profile velocity) around this handshake.
struct PpAxisConfig
{
  double counts_per_unit;
  double zero_counts;
  double min_position;
  double max_position;
  double position_tolerance;
  double stopped_velocity;
  double ack_timeout;
  double halt_timeout;
  unsigned int stopped_cycles;
};

struct PpAxisRequest
{
  uint64_t sequence;
  double position;
  bool halt;
};

struct PpFeedback
{
  bool operational;
  uint16_t control_word;
  uint16_t status_word;
  int mode;
  int32_t actual_position;
  double velocity;
};

struct PpAxisOutput
{
  uint16_t control_word{0};
  int32_t target{0};
  uint64_t sequence{0};
  PpState state{PpState::unavailable};
};

// New set-point handshake: bit 4 rises with the target, waits for set-point acknowledge
// (status bit 12), falls, and waits for the acknowledge to clear; then target reached
// (status bit 10) within tolerance. Halt (control bit 8) holds whenever no move runs.
class PpAxis
{
public:
  explicit PpAxis(const PpAxisConfig & config) : config_(config)
  {
    if (!valid_config(config)) {
      throw std::invalid_argument("PP axis requires verified calibration and limits");
    }
  }

  static bool valid_config(const PpAxisConfig & c) noexcept
  {
    const double low = c.zero_counts + c.min_position * c.counts_per_unit;
    const double high = c.zero_counts + c.max_position * c.counts_per_unit;
    return std::isfinite(c.counts_per_unit) && c.counts_per_unit != 0.0 &&
           std::isfinite(c.zero_counts) && std::isfinite(c.min_position) &&
           std::isfinite(c.max_position) && c.min_position >= 0.0 &&
           c.max_position > c.min_position && positive(c.position_tolerance) &&
           positive(c.stopped_velocity) && positive(c.ack_timeout) && positive(c.halt_timeout) &&
           c.stopped_cycles > 0 && raw_position_valid(low) && raw_position_valid(high);
  }

  double position(int32_t raw) const noexcept
  {
    return (static_cast<double>(raw) - config_.zero_counts) / config_.counts_per_unit;
  }

  // The adapter wrote this cycle's control word and target to the bus.
  void sent() noexcept { sent_ = true; }

  // True for the cycle in which a new target was taken; the adapter latches its own
  // per-target objects then.
  bool accepted() const noexcept { return accepted_; }

  // `extra_valid` carries the adapter's own checks of the request (for example a force limit).
  PpAxisOutput update(
    const PpAxisRequest & request, const PpFeedback & feedback, double now,
    bool extra_valid = true) noexcept
  {
    const bool sent = sent_;
    sent_ = false;
    accepted_ = false;
    const bool enabled = (feedback.control_word & 0x008fU) == 0x000fU;
    output_.control_word = feedback.control_word;
    if (!feedback.operational || !enabled) {
      output_.sequence = request.sequence;
      output_.target = feedback.actual_position;
      output_.state = PpState::unavailable;
      stopped_count_ = 0;
      last_time_ = now;
      // No PP overlay may turn a disable, quick-stop or fault-reset into enable.
      return enabled ? compose(feedback.control_word) : output_;
    }

    if (
      (feedback.status_word & 0x0008U) == 0 && !operation_enabled(feedback.status_word) &&
      output_.state != PpState::fault) {
      output_.sequence = request.sequence;
      output_.target = feedback.actual_position;
      output_.state = PpState::unavailable;
      return compose(feedback.control_word);
    }

    const bool healthy = operation_enabled(feedback.status_word) && feedback.mode == 1 &&
                         std::isfinite(feedback.velocity) && std::isfinite(now) &&
                         now >= last_time_ && (feedback.control_word & 0x0170U) == 0;
    last_time_ = now;
    if (!healthy) {
      output_.state = PpState::fault;
    }
    if (output_.state == PpState::unavailable) {
      output_.state = PpState::idle;
    }

    const bool idle = output_.state == PpState::idle || output_.state == PpState::reached ||
                      output_.state == PpState::halted;
    if (request.sequence != output_.sequence && output_.state != PpState::fault) {
      if (!idle || !valid_request(request) || !extra_valid) {
        output_.state = PpState::fault;
      } else {
        output_.sequence = request.sequence;
        output_.target = static_cast<int32_t>(
          std::llround(config_.zero_counts + request.position * config_.counts_per_unit));
        target_position_ = request.position;
        stopped_count_ = 0;
        accepted_ = true;
        enter(request.halt ? PpState::halting : PpState::preparing, now);
        return compose(feedback.control_word);
      }
    }

    if (
      request.halt && output_.sequence != 0 && output_.state != PpState::fault &&
      output_.state != PpState::idle && output_.state != PpState::halting &&
      output_.state != PpState::halted) {
      stopped_count_ = 0;
      enter(PpState::halting, now);
      return compose(feedback.control_word);
    }

    const bool ack = (feedback.status_word & 0x1000U) != 0;
    switch (output_.state) {
      case PpState::idle:
        output_.target = feedback.actual_position;
        break;
      case PpState::preparing:
        if (sent && !ack) {
          enter(PpState::waiting_ack, now);
        } else if (now - entered_ >= config_.ack_timeout) {
          output_.state = PpState::fault;
        }
        break;
      case PpState::waiting_ack:
        if (sent && ack) {
          enter(PpState::releasing_ack, now);
        } else if (now - entered_ >= config_.ack_timeout) {
          output_.state = PpState::fault;
        }
        break;
      case PpState::releasing_ack:
        if (sent && !ack) {
          enter(PpState::moving, now);
        } else if (now - entered_ >= config_.ack_timeout) {
          output_.state = PpState::fault;
        }
        break;
      case PpState::moving:
        if (
          (feedback.status_word & 0x0400U) != 0 &&
          std::abs(position(feedback.actual_position) - target_position_) <=
            config_.position_tolerance) {
          output_.state = PpState::reached;
        }
        break;
      case PpState::halting:
        stopped_count_ = sent && !ack && std::abs(feedback.velocity) <= config_.stopped_velocity
                           ? stopped_count_ + 1
                           : 0;
        if (stopped_count_ >= config_.stopped_cycles) {
          output_.state = PpState::halted;
        } else if (now - entered_ >= config_.halt_timeout) {
          output_.state = PpState::fault;
        }
        break;
      default:
        break;
    }
    return compose(feedback.control_word);
  }

  static bool positive(double value) noexcept { return std::isfinite(value) && value > 0.0; }

private:
  static bool operation_enabled(uint16_t status_word) noexcept
  {
    return hw_semantic_components::decode_cia402_state(status_word) ==
           hw_semantic_components::Cia402State::kOperationEnabled;
  }
  static bool raw_position_valid(double value) noexcept
  {
    return std::isfinite(value) && value >= std::numeric_limits<int32_t>::min() &&
           value <= std::numeric_limits<int32_t>::max();
  }
  bool valid_request(const PpAxisRequest & r) const noexcept
  {
    return r.sequence > 0 && std::isfinite(r.position) && r.position >= config_.min_position &&
           r.position <= config_.max_position;
  }
  void enter(PpState state, double now) noexcept
  {
    output_.state = state;
    entered_ = now;
  }
  PpAxisOutput compose(uint16_t base) noexcept
  {
    const bool run = output_.state == PpState::waiting_ack ||
                     output_.state == PpState::releasing_ack || output_.state == PpState::moving ||
                     output_.state == PpState::reached;
    output_.control_word = static_cast<uint16_t>(base & ~0x0170U);
    if (!run) {
      output_.control_word |= 0x0100U;
    }
    if (output_.state == PpState::waiting_ack) {
      output_.control_word |= 0x0010U;
    }
    return output_;
  }

  PpAxisConfig config_;
  PpAxisOutput output_;
  double entered_{0.0};
  double last_time_{0.0};
  double target_position_{0.0};
  unsigned int stopped_count_{0};
  bool sent_{false};
  bool accepted_{false};
};
}  // namespace hw_ethercat
#endif  // HW_ETHERCAT__PP_AXIS_HPP_
