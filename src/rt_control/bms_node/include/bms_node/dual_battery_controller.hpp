#ifndef BMS_NODE__DUAL_BATTERY_CONTROLLER_HPP_
#define BMS_NODE__DUAL_BATTERY_CONTROLLER_HPP_

#include "bms_node/protocol.hpp"

#include <cstdint>
#include <optional>

namespace bms_node
{

enum class DischargePhase
{
  kWaitingForRelayOpen,
  kWaitingForPrimary,
  kWaitingForSecondary,
  kWaitingForJoinConditions,
  kWaitingForAck,
  kWaitingForMos,
  kWaitingForRelayClosed,
  kObservingSecondary,
  kRunning,
  kStoppingLoads,
  kStoppingSecondary,
  kOpeningRelay,
  kStoppingPrimary,
  kFaultLatched,
  kAwaitingIsolationPolicy,
};

[[nodiscard]] const char * discharge_phase_name(DischargePhase phase) noexcept;

enum class DischargeAction
{
  kNone,
  kStopLoads,
  kEnableSecondary,
  kDisableSecondary,
  kDisablePrimary,
  kCloseRelay,
  kOpenRelay,
};

struct DischargeSettings
{
  double status_timeout_s;
  double command_timeout_s;
  double observation_s;
  double stable_s;
  double max_voltage_delta_v;
  double trip_voltage_delta_v;
  double max_abs_current_a;
  double min_cell_voltage_v;
  double max_cell_voltage_v;
  double min_cell_temperature_c;
  double max_cell_temperature_c;
  std::uint8_t mos_on_raw;
  std::uint8_t mos_off_raw;
};

struct DischargeInputs
{
  const PackState & primary;
  const PackState & secondary;
  double now_s;
  bool loads_stopped;
  std::optional<double> loads_stopped_s;
  std::optional<bool> relay_closed;
  std::optional<double> relay_feedback_s;
  bool control_authorized;
};

class DualBatteryController
{
public:
  explicit DualBatteryController(DischargeSettings settings);
  [[nodiscard]] DischargeAction update(const DischargeInputs & inputs);
  [[nodiscard]] DischargePhase phase() const noexcept {return phase_;}
  [[nodiscard]] bool secondary_faulted() const noexcept {return secondary_fault_;}

private:
  [[nodiscard]] bool healthy(const PackState & pack, double now_s) const noexcept;
  [[nodiscard]] bool join_conditions(const DischargeInputs & inputs) const noexcept;
  [[nodiscard]] bool join_interlocks(const DischargeInputs & inputs) const noexcept;
  [[nodiscard]] bool loads_safe_to_switch(const DischargeInputs & inputs) const noexcept;
  [[nodiscard]] bool feedback_fresh(const std::optional<double> & stamp, double now_s) const noexcept;
  void latch_fault(bool primary, bool secondary) noexcept;

  DischargeSettings settings_;
  DischargePhase phase_{DischargePhase::kWaitingForRelayOpen};
  std::optional<double> command_s_;
  std::optional<double> observation_s_;
  std::optional<double> stable_since_s_;
  bool primary_healthy_seen_{false};
  bool secondary_healthy_seen_{false};
  bool primary_fault_{false};
  bool secondary_fault_{false};
  bool primary_isolation_attempted_{false};
  bool secondary_isolation_attempted_{false};
};

}  // namespace bms_node

#endif  // BMS_NODE__DUAL_BATTERY_CONTROLLER_HPP_
