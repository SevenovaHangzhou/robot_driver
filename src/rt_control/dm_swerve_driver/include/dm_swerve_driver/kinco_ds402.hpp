#ifndef DM_SWERVE_DRIVER__KINCO_DS402_HPP_
#define DM_SWERVE_DRIVER__KINCO_DS402_HPP_

#include <cstdint>

namespace dm_swerve_driver {

enum class Ds402State {
  unknown,
  not_ready_to_switch_on,
  switch_on_disabled,
  ready_to_switch_on,
  switched_on,
  operation_enabled,
  quick_stop_active,
  fault_reaction_active,
  fault,
};

enum class FaultDisposition {
  none,
  recoverable,
  latch,
};

struct KincoFaultReport {
  FaultDisposition disposition{FaultDisposition::none};
  bool encoder{false};
  bool over_temperature{false};
  bool over_voltage{false};
  bool under_voltage{false};
  bool over_current{false};
  bool braking_resistor{false};
  bool following_error{false};
  bool overload{false};
  bool internal_or_configuration{false};
};

[[nodiscard]] Ds402State decode_ds402_state(std::uint16_t status_word) noexcept;
[[nodiscard]] std::uint16_t ds402_enable_control_word(Ds402State state) noexcept;
[[nodiscard]] KincoFaultReport classify_kinco_error_word(std::uint16_t error_word) noexcept;

class Ds402FaultResetSequence final {
public:
  void request() noexcept;
  [[nodiscard]] std::uint16_t next_control_word(Ds402State state) noexcept;
  [[nodiscard]] bool active() const noexcept;

private:
  enum class Phase {
    idle,
    low_before_pulse,
    pulse,
    low_after_pulse,
  };

  Phase phase_{Phase::idle};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__KINCO_DS402_HPP_
