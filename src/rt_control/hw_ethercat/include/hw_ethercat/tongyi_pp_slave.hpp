#ifndef HW_ETHERCAT__TONGYI_PP_SLAVE_HPP_
#define HW_ETHERCAT__TONGYI_PP_SLAVE_HPP_

#include <array>
#include <cstdint>
#include <memory>

#include "ethercat_generic_plugins/generic_ec_slave.hpp"
#include "hw_ethercat/pp_axis.hpp"

namespace hw_ethercat
{
// Tongyi XMC_ESC linear axis in Profile Position mode (V3-DOC-2026-001, ESI V3-DOC-2026-023).
// The drive's PDO layout is fixed, so this adapter owns every mapped output: 0x6060 = 1 and
// 0x607F (maximum profile velocity) are written each cycle, 0x6071/0x60FF stay 0, and 0x6081
// is the per-target profile velocity latched when the target is taken. The velocity state is
// the position derivative: the unit of 0x606C is not documented, so it is exported raw only.
class TongyiPpSlave final : public ethercat_generic_plugins::GenericEcSlave
{
public:
  bool setupSlave(
    std::unordered_map<std::string, std::string> parameters, std::vector<double> * states,
    std::vector<double> * commands) override;
  void processData(size_t index, uint8_t * address) override;
  void onPdoCycleRead(bool complete) override;
  void onPdoCycleStart(bool complete) override;
  void onPdoCycleSent() override;
  bool initialized() override { return preloaded_ && is_operational_; }
  void set_state_is_operational(bool operational) override;

  // Interface order of the rendered joint; the renderer emits the same names.
  static constexpr std::array<const char *, 5> kCommands{
    "position", "profile_velocity", "pp_sequence", "pp_halt", "control_word"};
  static constexpr std::array<const char *, 13> kStates{
    "position",
    "velocity",
    "status_word",
    "error_code",
    "mode_of_operation_display",
    "pp_sequence",
    "pp_state",
    "velocity_raw",
    "current_raw",
    "torque_raw",
    "bridge_temperature_raw",
    "motor_temperature_raw",
    "digital_inputs"};

private:
  enum Command : size_t { kPosition, kProfileVelocity, kSequence, kHalt, kControlWord };
  enum State : size_t {
    kStatePosition,
    kVelocity,
    kStatusWord,
    kErrorCode,
    kModeDisplay,
    kStateSequence,
    kPpState,
    kVelocityRaw,
    kCurrentRaw,
    kTorqueRaw,
    kBridgeTemperature,
    kMotorTemperature,
    kDigitalInputs
  };

  uint16_t base_control_word() const noexcept;
  double & state(State which) noexcept { return (*state_interface_ptr_)[states_[which]]; }

  std::unique_ptr<PpAxis> axis_;
  std::array<size_t, kCommands.size()> commands_{};
  std::array<size_t, kStates.size()> states_{};
  PpFeedback feedback_{false, 0, 0, 0, 0, 0.0};
  PpAxisOutput output_;
  double max_velocity_{0.0};
  double profile_velocity_per_mps_{0.0};
  uint32_t max_profile_velocity_{0};
  uint32_t profile_velocity_{0};
  int32_t previous_position_{0};
  double previous_time_{0.0};
  bool have_position_{false}, have_previous_{false}, preloaded_{false}, complete_{false};
  unsigned int written_{0};
};
}  // namespace hw_ethercat
#endif  // HW_ETHERCAT__TONGYI_PP_SLAVE_HPP_
