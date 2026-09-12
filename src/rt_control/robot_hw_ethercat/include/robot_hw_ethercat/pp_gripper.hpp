#ifndef ROBOT_HW_ETHERCAT__PP_GRIPPER_HPP_
#define ROBOT_HW_ETHERCAT__PP_GRIPPER_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace robot_hw_ethercat
{
// Values are internal ros2_control state, not a public ROS message contract.
enum class PpState : int
{
  fault = -1, unavailable = 0, idle = 1, preparing = 2, waiting_ack = 3,
  releasing_ack = 4, moving = 5, reached = 6, halting = 7, halted = 8
};

struct PpConfig
{
  double counts_per_metre;
  double zero_counts;
  double min_position;
  double max_position;
  double max_force;
  // A calibrated conservative conversion over the entire permitted stroke.
  double permille_per_newton;
  uint16_t max_torque_permille;
  double position_tolerance;
  double stopped_velocity;
  double ack_timeout;
  double halt_timeout;
  unsigned int stopped_cycles;
};

struct PpRequest
{
  uint64_t sequence;
  double position;
  double max_effort;
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

struct PpOutput
{
  uint16_t control_word{0};
  int32_t target{0};
  uint16_t max_torque{0};
  uint64_t sequence{0};
  PpState state{PpState::unavailable};
};

class PpGripper
{
public:
  explicit PpGripper(const PpConfig & config) : config_(config)
  {
    if (!valid_config(config)) {
      throw std::invalid_argument("PP gripper requires verified calibration and limits");
    }
  }

  static bool valid_config(const PpConfig & c) noexcept
  {
    const double low = c.zero_counts + c.min_position * c.counts_per_metre;
    const double high = c.zero_counts + c.max_position * c.counts_per_metre;
    return std::isfinite(c.counts_per_metre) && c.counts_per_metre != 0.0 &&
           std::isfinite(c.zero_counts) && std::isfinite(c.min_position) &&
           std::isfinite(c.max_position) && c.min_position >= 0.0 &&
           c.max_position > c.min_position && positive(c.max_force) &&
           positive(c.permille_per_newton) && c.max_torque_permille > 0 &&
           c.max_force * c.permille_per_newton <= c.max_torque_permille &&
           positive(c.position_tolerance) && positive(c.stopped_velocity) &&
           positive(c.ack_timeout) && positive(c.halt_timeout) && c.stopped_cycles > 0 &&
           raw_position_valid(low) && raw_position_valid(high);
  }

  double position(int32_t raw) const noexcept
  {
    return (static_cast<double>(raw) - config_.zero_counts) / config_.counts_per_metre;
  }

  void sent() noexcept {sent_ = true;}

  PpOutput update(const PpRequest & request, const PpFeedback & feedback, double now) noexcept
  {
    const bool sent = sent_;
    sent_ = false;
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

    if ((feedback.status_word & 0x0008U) == 0 &&
      (feedback.status_word & 0x006fU) != 0x0027U && output_.state != PpState::fault)
    {
      output_.sequence = request.sequence;
      output_.target = feedback.actual_position;
      output_.state = PpState::unavailable;
      return compose(feedback.control_word);
    }

    const bool healthy = (feedback.status_word & 0x006fU) == 0x0027U &&
      feedback.mode == 1 && std::isfinite(feedback.velocity) && std::isfinite(now) &&
      now >= last_time_ && (feedback.control_word & 0x0170U) == 0;
    last_time_ = now;
    if (!healthy) {output_.state = PpState::fault;}
    if (output_.state == PpState::unavailable) {output_.state = PpState::idle;}

    const bool idle = output_.state == PpState::idle || output_.state == PpState::reached ||
      output_.state == PpState::halted;
    if (request.sequence != output_.sequence && output_.state != PpState::fault) {
      if (!idle || !valid_request(request)) {
        output_.state = PpState::fault;
      } else {
        output_.sequence = request.sequence;
        output_.target = static_cast<int32_t>(std::llround(
          config_.zero_counts + request.position * config_.counts_per_metre));
        // Round downward: integer conversion must not increase the requested limit.
        output_.max_torque = static_cast<uint16_t>(
          std::floor(request.max_effort * config_.permille_per_newton));
        target_position_ = request.position;
        stopped_count_ = 0;
        enter(request.halt ? PpState::halting : PpState::preparing, now);
        return compose(feedback.control_word);
      }
    }

    if (request.halt && output_.sequence != 0 &&
      output_.state != PpState::fault && output_.state != PpState::idle &&
      output_.state != PpState::halting && output_.state != PpState::halted)
    {
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
        if (sent && !ack) {enter(PpState::waiting_ack, now);}
        else if (now - entered_ >= config_.ack_timeout) {output_.state = PpState::fault;}
        break;
      case PpState::waiting_ack:
        if (sent && ack) {enter(PpState::releasing_ack, now);}
        else if (now - entered_ >= config_.ack_timeout) {output_.state = PpState::fault;}
        break;
      case PpState::releasing_ack:
        if (sent && !ack) {enter(PpState::moving, now);}
        else if (now - entered_ >= config_.ack_timeout) {output_.state = PpState::fault;}
        break;
      case PpState::moving:
        if ((feedback.status_word & 0x0400U) != 0 &&
          std::abs(position(feedback.actual_position) - target_position_) <=
          config_.position_tolerance)
        {
          output_.state = PpState::reached;
        }
        break;
      case PpState::halting:
        stopped_count_ = sent && !ack &&
          std::abs(feedback.velocity) <= config_.stopped_velocity ? stopped_count_ + 1 : 0;
        if (stopped_count_ >= config_.stopped_cycles) {output_.state = PpState::halted;}
        else if (now - entered_ >= config_.halt_timeout) {output_.state = PpState::fault;}
        break;
      default:
        break;
    }
    return compose(feedback.control_word);
  }

private:
  static bool positive(double value) noexcept {return std::isfinite(value) && value > 0.0;}
  static bool raw_position_valid(double value) noexcept
  {
    return std::isfinite(value) && value >= std::numeric_limits<int32_t>::min() &&
           value <= std::numeric_limits<int32_t>::max();
  }
  bool valid_request(const PpRequest & r) const noexcept
  {
    return r.sequence > 0 && std::isfinite(r.position) &&
           r.position >= config_.min_position && r.position <= config_.max_position &&
           positive(r.max_effort) && r.max_effort <= config_.max_force &&
           r.max_effort * config_.permille_per_newton >= 1.0;
  }
  void enter(PpState state, double now) noexcept {output_.state = state; entered_ = now;}
  PpOutput compose(uint16_t base) noexcept
  {
    const bool run = output_.state == PpState::waiting_ack ||
      output_.state == PpState::releasing_ack || output_.state == PpState::moving ||
      output_.state == PpState::reached;
    output_.control_word = static_cast<uint16_t>(base & ~0x0170U);
    if (!run) {output_.control_word |= 0x0100U;}
    if (output_.state == PpState::waiting_ack) {output_.control_word |= 0x0010U;}
    return output_;
  }

  PpConfig config_;
  PpOutput output_;
  double entered_{0.0};
  double last_time_{0.0};
  double target_position_{0.0};
  unsigned int stopped_count_{0};
  bool sent_{false};
};
}  // namespace robot_hw_ethercat
#endif  // ROBOT_HW_ETHERCAT__PP_GRIPPER_HPP_
