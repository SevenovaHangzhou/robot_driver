#ifndef HW_ETHERCAT__PP_GRIPPER_HPP_
#define HW_ETHERCAT__PP_GRIPPER_HPP_

#include <cmath>
#include <cstdint>

#include "hw_ethercat/pp_axis.hpp"

namespace hw_ethercat
{
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

struct PpOutput
{
  uint16_t control_word{0};
  int32_t target{0};
  uint16_t max_torque{0};
  uint64_t sequence{0};
  PpState state{PpState::unavailable};
};

// PP gripper: the shared PP handshake plus a per-target force limit (0x6072).
class PpGripper
{
public:
  explicit PpGripper(const PpConfig & config) : config_(config), axis_(axis_config(config))
  {
    if (!valid_config(config)) {
      throw std::invalid_argument("PP gripper requires verified calibration and limits");
    }
  }

  static bool valid_config(const PpConfig & c) noexcept
  {
    return PpAxis::valid_config(axis_config(c)) && PpAxis::positive(c.max_force) &&
           PpAxis::positive(c.permille_per_newton) && c.max_torque_permille > 0 &&
           c.max_force * c.permille_per_newton <= c.max_torque_permille;
  }

  double position(int32_t raw) const noexcept { return axis_.position(raw); }

  void sent() noexcept { axis_.sent(); }

  PpOutput update(const PpRequest & request, const PpFeedback & feedback, double now) noexcept
  {
    const bool effort_valid = PpAxis::positive(request.max_effort) &&
                              request.max_effort <= config_.max_force &&
                              request.max_effort * config_.permille_per_newton >= 1.0;
    const auto axis = axis_.update(
      PpAxisRequest{request.sequence, request.position, request.halt}, feedback, now, effort_valid);
    if (axis_.accepted()) {
      // Round downward: integer conversion must not increase the requested limit.
      max_torque_ =
        static_cast<uint16_t>(std::floor(request.max_effort * config_.permille_per_newton));
    }
    return {axis.control_word, axis.target, max_torque_, axis.sequence, axis.state};
  }

private:
  static PpAxisConfig axis_config(const PpConfig & c) noexcept
  {
    return {c.counts_per_metre, c.zero_counts,        c.min_position,
            c.max_position,     c.position_tolerance, c.stopped_velocity,
            c.ack_timeout,      c.halt_timeout,       c.stopped_cycles};
  }

  PpConfig config_;
  PpAxis axis_;
  uint16_t max_torque_{0};
};
}  // namespace hw_ethercat
#endif  // HW_ETHERCAT__PP_GRIPPER_HPP_
