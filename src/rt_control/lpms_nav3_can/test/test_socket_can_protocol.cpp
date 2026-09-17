#include "lpms_nav3_can/socket_can.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <stdexcept>

TEST(SocketCanProtocolTest, BuildsExactFiltersForConfiguredNode)
{
  const auto filters = lpms_nav3_can::make_can_filters(7U);
  const std::array<canid_t, 5U> expected_ids{0x187U, 0x287U, 0x387U, 0x487U, 0x707U};

  for (std::size_t index = 0U; index < filters.size(); ++index) {
    EXPECT_EQ(filters[index].can_id, expected_ids[index]);
    EXPECT_EQ(filters[index].can_mask, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG);
  }
}

TEST(SocketCanProtocolTest, RejectsInvalidNodeIds)
{
  EXPECT_THROW((void)lpms_nav3_can::make_can_filters(0U), std::invalid_argument);
  EXPECT_THROW((void)lpms_nav3_can::make_can_filters(128U), std::invalid_argument);
}

TEST(SocketCanProtocolTest, ProtectsExistingRobotAndBmsInterfaces)
{
  EXPECT_TRUE(lpms_nav3_can::is_reserved_robot_interface("can0"));
  EXPECT_TRUE(lpms_nav3_can::is_reserved_robot_interface("can1"));
  EXPECT_FALSE(lpms_nav3_can::is_reserved_robot_interface("imu_can"));
  EXPECT_FALSE(lpms_nav3_can::is_reserved_robot_interface("vcan0"));
}
