#ifndef PERIPH_IMU_LPMS__SOCKET_CAN_HPP_
#define PERIPH_IMU_LPMS__SOCKET_CAN_HPP_

#include <linux/can.h>

#include <array>
#include <cstdint>

namespace periph_imu_lpms
{

[[nodiscard]] std::array<can_filter, 5U> make_can_filters(std::uint8_t node_id);

}  // namespace periph_imu_lpms

#endif  // PERIPH_IMU_LPMS__SOCKET_CAN_HPP_
