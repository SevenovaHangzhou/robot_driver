#ifndef ROBOT_HW_ETHERCAT__KINCO_CYCLIC_MODE_SLAVE_HPP_
#define ROBOT_HW_ETHERCAT__KINCO_CYCLIC_MODE_SLAVE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "ethercat_generic_plugins/generic_ec_slave.hpp"

namespace robot_hw_ethercat
{
class KincoCyclicModeSlave final : public ethercat_generic_plugins::GenericEcSlave
{
public:
  bool setupSlave(
    std::unordered_map<std::string, std::string> parameters,
    std::vector<double> * states, std::vector<double> * commands) override;
  void processData(size_t index, uint8_t * address) override;
  void onPdoCycleRead(bool complete) override;
  void onPdoCycleStart(bool complete) override;
  void onPdoCycleSent() override;
  bool initialized() override;
  void set_state_is_operational(bool operational) override;

private:
  static constexpr int8_t kCsp = 8;
  static constexpr int8_t kCsv = 9;
  static constexpr uint8_t kAllOutputs = 0x0fU;

  enum Command : size_t {kPosition, kVelocity, kMode, kControlWord, kWriteSequence, kWriteMask,
    kCommandCount};
  enum State : size_t
  {
    kActualPosition,
    kActualVelocity,
    kModeDisplay,
    kStatusWord,
    kFeedbackAge,
    kModeSwitchReady,
    kModeRequestAck,
    kCommandFresh,
    kModeRequestError,
    kFeedbackSequence, kSentSequence, kFeedbackSequenceAtSend, kSentVelocity,
    kStateCount
  };

  bool feedback_fresh(std::chrono::steady_clock::time_point now) const noexcept;
  bool operation_enabled() const noexcept;
  bool valid_position_command(int32_t & raw) const noexcept;
  bool valid_velocity_command(int32_t & raw) const noexcept;
  int requested_mode() const noexcept;
  uint16_t requested_control_word() const noexcept;
  void clear_handoff(bool error) noexcept;
  void set_state(State state, double value) noexcept;
  void invalidate_velocity_command() noexcept;

  std::array<size_t, kCommandCount> commands_{};
  std::array<size_t, kStateCount> states_{};
  double position_counts_per_unit_{0.0};
  double velocity_counts_per_unit_{0.0};
  double position_offset_counts_{0.0};
  double min_position_{0.0};
  double max_position_{0.0};
  double max_abs_velocity_{0.0};
  double stationary_velocity_{0.0};
  double feedback_timeout_seconds_{0.0};
  double mode_ack_timeout_seconds_{0.0};
  int8_t startup_mode_{0};

  int32_t actual_position_raw_{0};
  int32_t actual_velocity_raw_{0};
  uint16_t status_word_{0};
  int8_t mode_display_{0};
  int32_t output_position_raw_{0};
  int32_t output_velocity_raw_{0};
  int32_t csp_seed_raw_{0};
  int32_t csv_hold_raw_{0};
  uint16_t output_control_word_{0};
  int8_t output_mode_{0};

  std::chrono::steady_clock::time_point last_complete_feedback_{};
  std::chrono::steady_clock::time_point mode_request_started_{};
  bool read_hook_seen_{false};
  bool cycle_complete_{false};
  bool have_position_{false};
  bool have_velocity_{false};
  bool have_status_{false};
  bool have_mode_{false};
  bool startup_seed_pending_{false};
  bool startup_seed_sent_{false};
  bool csp_preseed_pending_{false};
  bool csp_preseed_sent_{false};
  bool mode_request_in_flight_{false};
  bool csv_zero_pending_{false};
  bool csv_requires_fresh_command_{true};
  bool csv_post_ack_quarantine_{false};
  bool error_latched_{false};
  int8_t mode_request_target_{0};
  uint8_t written_outputs_{0};
  // Double interfaces carry exact integers only, up to 2^53-1. Ack is recorded once
  // per sequence AFTER ecrt_master_send(), never on assignment or mode readback.
  uint64_t feedback_sequence_{0}, sent_sequence_{0}, pending_sequence_{0};
  uint8_t pending_mask_{0};
  int32_t pending_position_{0}, pending_velocity_{0};
  int pending_mode_{0};
  bool pending_valid_{false};
};
}  // namespace robot_hw_ethercat

#endif  // ROBOT_HW_ETHERCAT__KINCO_CYCLIC_MODE_SLAVE_HPP_
