#pragma once

#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

namespace hw_ethercat
{
// Preserve the upstream CSP position/state machine. Only the additive current
// channel is gated here; it must never retain a previous session's feedforward.
class CspTorqueOffsetSlave final : public ethercat_generic_plugins::EcCiA402Drive
{
public:
  bool setupSlave(
    std::unordered_map<std::string, std::string> parameters, std::vector<double> * states,
    std::vector<double> * commands) override;
  void processData(size_t index, uint8_t * address) override;
  void onPdoCycleStart(bool complete) override;
  void set_state_is_operational(bool operational) override;

private:
  void clear_offset() noexcept;
  int offset_command_index_{-1};
  int control_command_index_{-1};
  int raw_state_index_{-1};
  int epoch_state_index_{-1};
  double command_epoch_{0.0};
  bool complete_{false};
};
}  // namespace hw_ethercat
