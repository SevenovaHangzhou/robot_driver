#ifndef ROBOT_HW_CAN__HEAD_STATE_MACHINE_HPP_
#define ROBOT_HW_CAN__HEAD_STATE_MACHINE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

namespace robot_hw_can
{

enum class HeadPhase : std::uint8_t
{
  disabled = 0U,
  enabling = 1U,
  enabled = 2U,
  disabling = 3U,
  reset_disabling = 4U,
  resetting = 5U,
  fault_latched = 6U,
};

struct MotorStatus
{
  bool valid{false};
  std::uint8_t status{0U};
  std::chrono::nanoseconds received_at{0};
};

struct HeadActions
{
  bool seed_hold{false};
  bool send_hold{false};
  bool send_enable{false};
  bool send_disable{false};
  bool send_clear_fault{false};
  bool allow_motion{false};
};

class HeadStateMachine
{
public:
  HeadStateMachine(
    std::chrono::nanoseconds feedback_timeout,
    std::chrono::nanoseconds transition_timeout);

  [[nodiscard]] HeadActions step(
    std::chrono::nanoseconds now, bool enable_requested,
    std::uint64_t reset_generation,
    const std::array<MotorStatus, 2U> & motors);
  [[nodiscard]] HeadPhase phase() const noexcept;
  [[nodiscard]] bool fault_latched() const noexcept;
  [[nodiscard]] std::uint64_t handled_reset_generation() const noexcept;
  [[nodiscard]] HeadActions force_fault() noexcept;

private:
  [[nodiscard]] bool all_fresh(
    std::chrono::nanoseconds now,
    const std::array<MotorStatus, 2U> & motors) const noexcept;
  [[nodiscard]] bool all_received_after_transition(
    const std::array<MotorStatus, 2U> & motors) const noexcept;
  void transition(HeadPhase phase, std::chrono::nanoseconds now) noexcept;
  [[nodiscard]] HeadActions latch_fault() noexcept;

  std::chrono::nanoseconds feedback_timeout_;
  std::chrono::nanoseconds transition_timeout_;
  std::chrono::nanoseconds transition_started_at_{0};
  HeadPhase phase_{HeadPhase::disabled};
  bool fault_latched_{false};
  std::uint64_t handled_reset_generation_{0U};
  std::uint64_t pending_reset_generation_{0U};
};

class CommandRateLimiter
{
public:
  explicit CommandRateLimiter(std::chrono::nanoseconds period);

  [[nodiscard]] bool due(std::chrono::nanoseconds now) noexcept;
  void reset() noexcept;

private:
  std::chrono::nanoseconds period_;
  std::optional<std::chrono::nanoseconds> last_emitted_at_;
};

}  // namespace robot_hw_can

#endif  // ROBOT_HW_CAN__HEAD_STATE_MACHINE_HPP_
