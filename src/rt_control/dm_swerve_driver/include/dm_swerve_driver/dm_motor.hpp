#ifndef DM_SWERVE_DRIVER__DM_MOTOR_HPP_
#define DM_SWERVE_DRIVER__DM_MOTOR_HPP_

#include <cstdint>

#include "dm_swerve_driver/dm_frame_codec.hpp"

namespace dm_swerve_driver {

struct DmMotorConfig {
  std::uint16_t esc_id{0U};
  std::uint16_t mst_id{0U};
  MotorLimits limits{};
};

struct DmMotorHealth {
  bool has_feedback{false};
  bool seeded_from_multi_turn{false};
  MotorError error{MotorError::disabled};
  std::uint8_t mos_temperature_c{0U};
  std::uint8_t rotor_temperature_c{0U};
  std::uint64_t received_frames{0U};
  std::uint64_t missed_frames{0U};
  std::uint64_t consecutive_missed_frames{0U};

  [[nodiscard]] bool enabled() const noexcept;
  [[nodiscard]] bool has_fault() const noexcept;
};

class DmMotor final {
public:
  explicit DmMotor(DmMotorConfig config);

  [[nodiscard]] std::uint16_t esc_id() const noexcept;
  [[nodiscard]] std::uint16_t mst_id() const noexcept;
  [[nodiscard]] const MotorLimits & limits() const noexcept;

  void set_limits(const MotorLimits & limits);
  void seed_position(double raw_position);
  [[nodiscard]] bool seed_position_from_multi_turn(
    double raw_position, double multi_turn_position,
    double consistency_tolerance_rad = 0.1);

  [[nodiscard]] static bool multi_turn_consistent(
    double raw_position, double multi_turn_position,
    double tolerance_rad = 0.1);

  [[nodiscard]] MotorFeedback accept_feedback(const CanFrame & frame);
  void mark_feedback_missed() noexcept;

  [[nodiscard]] bool position_initialized() const noexcept;
  [[nodiscard]] double raw_position() const noexcept;
  [[nodiscard]] double unwrapped_position() const noexcept;
  [[nodiscard]] double velocity() const noexcept;
  [[nodiscard]] double torque() const noexcept;
  [[nodiscard]] const DmMotorHealth & health() const noexcept;

  [[nodiscard]] CanFrame encode_command(const MitCommand & command) const;
  [[nodiscard]] CanFrame special_command(SpecialCommand command) const;

private:
  void update_position(double raw_position) noexcept;

  DmMotorConfig config_;
  DmMotorHealth health_{};
  bool position_initialized_{false};
  double raw_position_{0.0};
  double position_offset_{0.0};
  double unwrapped_position_{0.0};
  double velocity_{0.0};
  double torque_{0.0};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__DM_MOTOR_HPP_
