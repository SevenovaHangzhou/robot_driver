#ifndef ROBOT_HW_CAN__PROTOCOL_HPP_
#define ROBOT_HW_CAN__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace robot_hw_can
{

inline constexpr std::size_t kPayloadSize{8U};
inline constexpr std::uint16_t kRegisterRequestId{0x7FFU};
inline constexpr std::uint8_t kPositionVelocityMode{2U};
inline constexpr std::uint32_t kOneMegabitCanCode{4U};

struct CanFrame
{
  std::uint16_t id{0U};
  std::uint8_t length{static_cast<std::uint8_t>(kPayloadSize)};
  std::array<std::uint8_t, kPayloadSize> data{};
};

struct MotorLimits
{
  double position_max{0.0};
  double velocity_max{0.0};
  double torque_max{0.0};

  [[nodiscard]] bool valid() const noexcept;
};

struct TrapezoidalProfile
{
  double acceleration_krad_s2{0.0};
  double deceleration_krad_s2{0.0};
  double maximum_speed_rad_s{0.0};

  [[nodiscard]] bool valid() const noexcept;
};

struct MotorFeedback
{
  std::uint8_t motor_id{0U};
  std::uint8_t status{0U};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double mos_temperature_c{0.0};
  double motor_temperature_c{0.0};
};

enum class SpecialCommand : std::uint8_t
{
  clear_fault = 0xFBU,
  enable = 0xFCU,
  disable = 0xFDU,
};

enum class Register : std::uint8_t
{
  acceleration = 0x04U,
  deceleration = 0x05U,
  maximum_speed = 0x06U,
  master_id = 0x07U,
  can_id = 0x08U,
  can_timeout = 0x09U,
  control_mode = 0x0AU,
  position_max = 0x15U,
  velocity_max = 0x16U,
  torque_max = 0x17U,
  can_bitrate = 0x23U,
};

struct MotorConfiguration
{
  std::uint32_t can_id{0U};
  std::uint32_t master_id{0U};
  std::uint32_t timeout_ticks{0U};
  std::uint32_t bitrate_code{0U};
};

struct RegisterReply
{
  std::uint16_t motor_id{0U};
  std::uint8_t operation{0U};
  std::uint8_t register_id{0U};
  std::uint32_t raw_value{0U};

  [[nodiscard]] float float_value() const noexcept;
};

[[nodiscard]] CanFrame make_position_velocity_command(
  std::uint16_t motor_id, float position_rad, float velocity_limit_rad_s);
[[nodiscard]] CanFrame make_special_command(
  std::uint16_t motor_id, SpecialCommand command);
[[nodiscard]] CanFrame make_register_read(std::uint16_t motor_id, Register register_id);
[[nodiscard]] CanFrame make_register_write_float(
  std::uint16_t motor_id, Register register_id, float value);
[[nodiscard]] CanFrame make_parameter_save(std::uint16_t motor_id);
[[nodiscard]] RegisterReply decode_register_reply(const CanFrame & frame);
void validate_motor_configuration(
  const MotorConfiguration & configuration, std::uint16_t expected_can_id,
  std::uint16_t expected_master_id);
void validate_trapezoidal_profile(
  const TrapezoidalProfile & actual, const TrapezoidalProfile & expected);
[[nodiscard]] MotorFeedback decode_feedback(
  const CanFrame & frame, const MotorLimits & limits);

}  // namespace robot_hw_can

#endif  // ROBOT_HW_CAN__PROTOCOL_HPP_
