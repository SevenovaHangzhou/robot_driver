#ifndef DM_SWERVE_DRIVER__DM_FRAME_CODEC_HPP_
#define DM_SWERVE_DRIVER__DM_FRAME_CODEC_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace dm_swerve_driver {

inline constexpr std::uint16_t kRegisterCanId{0x7FFU};
inline constexpr std::size_t kCanPayloadSize{8U};
inline constexpr double kMitKpMax{500.0};
inline constexpr double kMitKdMax{5.0};

struct CanFrame {
  std::uint16_t id{0U};
  std::uint8_t length{static_cast<std::uint8_t>(kCanPayloadSize)};
  std::array<std::uint8_t, kCanPayloadSize> data{};
};

struct MotorLimits {
  double position_max{0.0};
  double velocity_max{0.0};
  double torque_max{0.0};

  [[nodiscard]] bool valid() const noexcept;
};

struct MitCommand {
  double position{0.0};
  double velocity{0.0};
  double kp{0.0};
  double kd{0.0};
  double torque{0.0};
};

enum class MotorError : std::uint8_t {
  disabled = 0x0U,
  enabled = 0x1U,
  encoder = 0x2U,
  encoder_read = 0x5U,
  over_voltage = 0x8U,
  under_voltage = 0x9U,
  over_current = 0xAU,
  mos_over_temperature = 0xBU,
  coil_over_temperature = 0xCU,
  communication_lost = 0xDU,
  overload = 0xEU,
};

struct MotorFeedback {
  std::uint8_t motor_id{0U};
  MotorError error{MotorError::disabled};
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  std::uint8_t mos_temperature_c{0U};
  std::uint8_t rotor_temperature_c{0U};
};

enum class SpecialCommand : std::uint8_t {
  clear_fault = 0xFBU,
  enable = 0xFCU,
  disable = 0xFDU,
  save_zero = 0xFEU,
};

enum class RegisterOperation : std::uint8_t {
  read = 0x33U,
  write = 0x55U,
};

enum class RegisterId : std::uint8_t {
  timeout = 0x09U,
  position_max = 0x15U,
  velocity_max = 0x16U,
  torque_max = 0x17U,
  multi_turn_position = 0x50U,
};

struct RegisterReply {
  std::uint16_t motor_can_id{0U};
  RegisterOperation operation{RegisterOperation::read};
  std::uint8_t register_id{0U};
  std::uint32_t raw_value{0U};

  [[nodiscard]] float float_value() const noexcept;
};

[[nodiscard]] std::uint32_t float_to_uint(
  double value, double minimum, double maximum, unsigned int bits);
[[nodiscard]] double uint_to_float(
  std::uint32_t value, double minimum, double maximum, unsigned int bits);

[[nodiscard]] CanFrame encode_mit_command(
  std::uint16_t esc_id, const MitCommand & command, const MotorLimits & limits);
[[nodiscard]] MitCommand decode_mit_command(
  const CanFrame & frame, const MotorLimits & limits);
[[nodiscard]] CanFrame encode_motor_feedback(
  std::uint16_t mst_id, const MotorFeedback & feedback, const MotorLimits & limits);
[[nodiscard]] MotorFeedback decode_motor_feedback(
  const CanFrame & frame, const MotorLimits & limits);

[[nodiscard]] CanFrame make_special_command(
  std::uint16_t esc_id, SpecialCommand command);
[[nodiscard]] std::optional<SpecialCommand> decode_special_command(
  const CanFrame & frame) noexcept;

[[nodiscard]] CanFrame make_register_read(
  std::uint16_t motor_can_id, std::uint8_t register_id);
[[nodiscard]] CanFrame make_register_write(
  std::uint16_t motor_can_id, std::uint8_t register_id, std::uint32_t raw_value);
[[nodiscard]] CanFrame make_register_write_float(
  std::uint16_t motor_can_id, std::uint8_t register_id, float value);
[[nodiscard]] RegisterReply decode_register_reply(const CanFrame & frame);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__DM_FRAME_CODEC_HPP_
