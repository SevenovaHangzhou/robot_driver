#include <gtest/gtest.h>

#include "bms_node/can_transport.hpp"

#include <linux/can.h>

#include <cstddef>
#include <cstdint>

namespace
{

constexpr std::uint32_t kRequestId = 0x18900140U;
constexpr std::uint32_t kResponseId = 0x18904001U;

TEST(CanTransportTest, FilterMatchesOnlyExactExtendedDataResponse)
{
  const auto filter = bms_node::make_response_filter(kResponseId);
  EXPECT_EQ(filter.can_id, static_cast<canid_t>(kResponseId) | CAN_EFF_FLAG);
  EXPECT_EQ(
    filter.can_mask,
    static_cast<canid_t>(CAN_EFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG));
}

TEST(CanTransportTest, QueryIsAnEightByteZeroFilledExtendedFrame)
{
  const auto frame = bms_node::make_query_frame(kRequestId);
  EXPECT_EQ(frame.can_id, static_cast<canid_t>(kRequestId) | CAN_EFF_FLAG);
  EXPECT_EQ(frame.can_dlc, CAN_MAX_DLEN);
  for (const auto value : frame.data) {
    EXPECT_EQ(value, 0U);
  }
}

TEST(CanTransportTest, DecodesOnlyExactEightByteExtendedDataResponse)
{
  can_frame frame{};
  frame.can_id = static_cast<canid_t>(kResponseId) | CAN_EFF_FLAG;
  frame.can_dlc = CAN_MAX_DLEN;
  frame.data[0] = 0x12U;
  const std::uint8_t * data = nullptr;
  std::size_t size = 0U;

  ASSERT_TRUE(bms_node::decode_response_frame(frame, kResponseId, data, size));
  ASSERT_NE(data, nullptr);
  EXPECT_EQ(size, 8U);
  EXPECT_EQ(data[0], 0x12U);

  frame.can_id = kResponseId;
  EXPECT_FALSE(bms_node::decode_response_frame(frame, kResponseId, data, size));
  frame.can_id = static_cast<canid_t>(kResponseId) | CAN_EFF_FLAG | CAN_RTR_FLAG;
  EXPECT_FALSE(bms_node::decode_response_frame(frame, kResponseId, data, size));
  frame.can_id = static_cast<canid_t>(kResponseId + 1U) | CAN_EFF_FLAG;
  EXPECT_FALSE(bms_node::decode_response_frame(frame, kResponseId, data, size));
  frame.can_id = static_cast<canid_t>(kResponseId) | CAN_EFF_FLAG;
  frame.can_dlc = 7U;
  EXPECT_FALSE(bms_node::decode_response_frame(frame, kResponseId, data, size));
}

}  // namespace
