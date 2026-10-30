#include "periph_imu_lpms/protocol/socket_can.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <stdexcept>

TEST(SocketCanProtocolTest, BuildsExactFiltersForConfiguredNode)
{
  const auto filters = periph_imu_lpms::make_can_filters(7U);
  const std::array<canid_t, 5U> expected_ids{0x187U, 0x287U, 0x387U, 0x487U, 0x707U};

  for (std::size_t index = 0U; index < filters.size(); ++index) {
    EXPECT_EQ(filters[index].can_id, expected_ids[index]);
    EXPECT_EQ(filters[index].can_mask, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG);
  }
}

TEST(SocketCanProtocolTest, RejectsInvalidNodeIds)
{
  EXPECT_THROW((void)periph_imu_lpms::make_can_filters(0U), std::invalid_argument);
  EXPECT_THROW((void)periph_imu_lpms::make_can_filters(128U), std::invalid_argument);
}
