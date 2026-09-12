#ifndef DM_SWERVE_DRIVER__CAN_FRAME_HPP_
#define DM_SWERVE_DRIVER__CAN_FRAME_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace dm_swerve_driver {

inline constexpr std::size_t kCanPayloadSize{8U};

struct CanFrame {
  std::uint16_t id{0U};
  std::uint8_t length{static_cast<std::uint8_t>(kCanPayloadSize)};
  std::array<std::uint8_t, kCanPayloadSize> data{};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__CAN_FRAME_HPP_
