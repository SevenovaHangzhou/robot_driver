#ifndef GRAVITY_FF_CONTROLLER__CONTROL_CORE_HPP_
#define GRAVITY_FF_CONTROLLER__CONTROL_CORE_HPP_

#include <cstdint>
#include <vector>

namespace gravity_ff_controller
{
enum class DriveState : std::uint8_t
{
  kNotReady,
  kSwitchOnDisabled,
  kReady,
  kSwitchedOn,
  kOperationEnabled,
  kQuickStop,
  kFaultReaction,
  kFault,
  kUnknown
};
enum class StepStatus : std::uint8_t
{
  kOk,
  kDisableTransitionHold,
  kFaultReactionHold,
  kFaultLatched,
  kInvalidInputLatched,
  kAlreadyLatched
};

struct Limits
{
  std::vector<double> maximum_effort_nm;
  std::vector<double> maximum_slew_nm_per_s;
  double fault_slew_nm_per_s{1000.0};
};

class ControlCore final
{
public:
  bool configure(const Limits & limits) noexcept;
  void activate() noexcept;
  StepStatus step(
    const std::vector<double> & gravity_nm,
    const std::vector<double> & scale,
    const std::vector<double> & status_words, bool model_valid,
    double period_seconds) noexcept;
  void force_zero() noexcept;
  bool fault_latched() const noexcept {return fault_latched_;}
  const std::vector<double> & output_nm() const noexcept {return output_nm_;}
  DriveState last_valid_state() const noexcept {return last_valid_state_;}
  static DriveState decode(double status_word) noexcept;

private:
  void ramp_to(
    const std::vector<double> & desired,
    double period_seconds) noexcept;
  void fault_ramp(double period_seconds) noexcept;
  Limits limits_;
  std::vector<double> output_nm_, desired_nm_;
  bool configured_{false};
  bool fault_latched_{false};
  bool fault_ramp_active_{false};
  DriveState last_valid_state_{DriveState::kUnknown};
};
} // namespace gravity_ff_controller
#endif // GRAVITY_FF_CONTROLLER__CONTROL_CORE_HPP_
