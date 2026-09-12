#ifndef DM_SWERVE_DRIVER__KINCO_PDO_HPP_
#define DM_SWERVE_DRIVER__KINCO_PDO_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace dm_swerve_driver {

inline constexpr std::size_t kKincoAxisCount{8U};

enum class KincoOperationMode : std::int8_t {
  cyclic_synchronous_position = 8,
  cyclic_synchronous_velocity = 9,
  cyclic_synchronous_torque = 10,
};

struct CoePdoEntry {
  std::uint16_t index{0U};
  std::uint8_t subindex{0U};
  std::uint8_t bit_length{0U};
};

inline constexpr CoePdoEntry kKincoControlword{0x6040U, 0U, 16U};
inline constexpr CoePdoEntry kKincoModeOfOperation{0x6060U, 0U, 8U};
inline constexpr CoePdoEntry kKincoTargetPosition{0x607AU, 0U, 32U};
inline constexpr CoePdoEntry kKincoTargetVelocity{0x60FFU, 0U, 32U};
inline constexpr CoePdoEntry kKincoTargetTorque{0x6071U, 0U, 16U};

inline constexpr CoePdoEntry kKincoStatusword{0x6041U, 0U, 16U};
inline constexpr CoePdoEntry kKincoModeDisplay{0x6061U, 0U, 8U};
inline constexpr CoePdoEntry kKincoActualPosition{0x6064U, 0U, 32U};
inline constexpr CoePdoEntry kKincoActualVelocity{0x606CU, 0U, 32U};
inline constexpr CoePdoEntry kKincoActualTorque{0x6077U, 0U, 16U};
inline constexpr CoePdoEntry kKincoErrorWord{0x2601U, 0U, 16U};

inline constexpr std::array<CoePdoEntry, 5U> kKincoRxPdoEntries{
  kKincoControlword,
  kKincoModeOfOperation,
  kKincoTargetPosition,
  kKincoTargetVelocity,
  kKincoTargetTorque};

inline constexpr std::array<CoePdoEntry, 6U> kKincoTxPdoEntries{
  kKincoStatusword,
  kKincoModeDisplay,
  kKincoActualPosition,
  kKincoActualVelocity,
  kKincoActualTorque,
  kKincoErrorWord};

struct KincoAxisCommand {
  std::uint16_t control_word{0U};
  KincoOperationMode mode{KincoOperationMode::cyclic_synchronous_position};
  std::int32_t target_position{0};
  std::int32_t target_velocity{0};
  std::int16_t target_torque_percent{0};
};

struct KincoAxisFeedback {
  std::uint16_t status_word{0U};
  std::int8_t mode_display{0};
  std::int32_t actual_position{0};
  std::int32_t actual_velocity{0};
  std::int16_t actual_torque_percent{0};
  std::uint16_t error_word{0U};
  bool online{false};
  std::uint16_t extended_error_word{0U};
};

using KincoCommandBatch = std::array<KincoAxisCommand, kKincoAxisCount>;
using KincoFeedbackBatch = std::array<KincoAxisFeedback, kKincoAxisCount>;

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__KINCO_PDO_HPP_
