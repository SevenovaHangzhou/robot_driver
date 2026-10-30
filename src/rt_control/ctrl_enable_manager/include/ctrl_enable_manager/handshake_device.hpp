#ifndef CTRL_ENABLE_MANAGER__HANDSHAKE_DEVICE_HPP_
#define CTRL_ENABLE_MANAGER__HANDSHAKE_DEVICE_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rt_control_semantic_components/handshake_phase.hpp"

namespace ctrl_enable_manager
{

// A device whose hardware plugin runs its own enable sequence (e.g. the DaMiao head).
// The controller only requests enable/disable and a fault reset through a control GPIO,
// and reads back the plugin's phase:
//   commands: <gpio>/enable_request (0|1), <gpio>/reset_generation (exact integer)
//   states:   <gpio>/phase, <gpio>/fault_latched (0|1),
//             <gpio>/handled_reset_generation, <gpio>/motion_allowed (0|1)
// read()/write() run in the real-time update; status() is safe from other threads.
class HandshakeDevice
{
public:
  using Phase = rt_control_semantic_components::HandshakePhase;

  struct Status
  {
    bool valid{false};
    Phase phase{Phase::disabled};
    bool fault_latched{false};
    bool motion_allowed{false};
  };

  explicit HandshakeDevice(std::string gpio);

  [[nodiscard]] const std::string & name() const noexcept { return gpio_; }
  [[nodiscard]] std::vector<std::string> command_interface_names() const;
  [[nodiscard]] std::vector<std::string> state_interface_names() const;

  bool bind(
    std::vector<hardware_interface::LoanedCommandInterface> & commands,
    std::vector<hardware_interface::LoanedStateInterface> & states);
  void release() noexcept;

  // Activation: no enable request, and the reset counter continues from the plugin's value.
  bool baseline() noexcept;
  void read() noexcept;
  void write() noexcept;

  void request_enable(bool enable) noexcept { enable_requested_ = enable; }
  void start_reset() noexcept;

  // An unreadable sample counts as a fault: the conservative reaction is to stop.
  [[nodiscard]] bool faulted() const noexcept;
  [[nodiscard]] bool enabled() const noexcept;
  [[nodiscard]] bool disabled() const noexcept;
  [[nodiscard]] bool reset_complete() const noexcept;
  [[nodiscard]] Status status() const noexcept;

private:
  std::string gpio_;
  hardware_interface::LoanedCommandInterface * enable_command_{nullptr};
  hardware_interface::LoanedCommandInterface * reset_command_{nullptr};
  std::array<hardware_interface::LoanedStateInterface *, 4U> states_{};

  bool enable_requested_{false};
  std::uint64_t reset_generation_{0U};
  std::uint64_t reset_target_{0U};
  bool reset_pending_{false};

  bool valid_{false};
  Phase phase_{Phase::disabled};
  bool fault_latched_{false};
  bool motion_allowed_{false};
  std::uint64_t handled_generation_{0U};

  std::atomic<std::uint32_t> published_status_{0U};
};

}  // namespace ctrl_enable_manager

#endif  // CTRL_ENABLE_MANAGER__HANDSHAKE_DEVICE_HPP_
