#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "dm_swerve_driver/canopen_encoder.hpp"

namespace dm_swerve_driver {
namespace {

using namespace std::chrono_literals;

[[nodiscard]] CanFrame sdo_reply(
  std::uint8_t node_id,
  std::uint16_t index,
  std::uint8_t subindex,
  std::uint32_t value,
  std::uint8_t command = 0x43U)
{
  CanFrame frame;
  frame.id = static_cast<std::uint16_t>(0x580U + node_id);
  frame.data[0] = command;
  frame.data[1] = static_cast<std::uint8_t>(index & 0xFFU);
  frame.data[2] = static_cast<std::uint8_t>((index >> 8U) & 0xFFU);
  frame.data[3] = subindex;
  for (std::size_t byte{0U}; byte < 4U; ++byte) {
    frame.data[4U + byte] = static_cast<std::uint8_t>((value >> (8U * byte)) & 0xFFU);
  }
  return frame;
}

[[nodiscard]] CanFrame tpdo_reply(std::uint8_t node_id, std::uint32_t position)
{
  CanFrame frame;
  frame.id = static_cast<std::uint16_t>(0x280U + node_id);
  frame.length = 4U;
  for (std::size_t byte{0U}; byte < 4U; ++byte) {
    frame.data[byte] = static_cast<std::uint8_t>((position >> (8U * byte)) & 0xFFU);
  }
  return frame;
}

class FakeEncoderTransport final : public CanTransport {
public:
  void open() override {open_ = true;}
  void close() noexcept override {open_ = false;}
  [[nodiscard]] bool is_open() const noexcept override {return open_;}

  void write_batch(const std::vector<CanFrame> & frames) override
  {
    batches_.push_back(frames);
    for (const auto & frame : frames) {
      if (frame.id >= 0x601U && frame.id <= 0x67FU && frame.data[0] == 0x40U) {
        const auto node = static_cast<std::uint8_t>(frame.id - 0x600U);
        const auto index = static_cast<std::uint16_t>(
          frame.data[1] | static_cast<std::uint16_t>(frame.data[2] << 8U));
        const std::uint32_t value = index == 0x6501U ? 65536U : 24U;
        pending_.push_front(ReceivedCanFrame{sdo_reply(node, index, frame.data[3], value), {}});
      } else if (frame.id == 0x080U) {
        for (auto iter = positions_.rbegin(); iter != positions_.rend(); ++iter) {
          const std::size_t index{static_cast<std::size_t>(iter - positions_.rbegin())};
          const std::uint8_t node{static_cast<std::uint8_t>(4U - index)};
          if (!drop_node_.has_value() || node != *drop_node_) {
            pending_.push_back(ReceivedCanFrame{tpdo_reply(node, *iter), {}});
          }
        }
      }
    }
  }

  [[nodiscard]] std::vector<ReceivedCanFrame> collect(
    std::size_t expected_count,
    std::chrono::steady_clock::time_point) override
  {
    std::vector<ReceivedCanFrame> result;
    while (!pending_.empty() && result.size() < expected_count) {
      result.push_back(pending_.front());
      pending_.pop_front();
    }
    return result;
  }

  void set_drop_node(std::uint8_t node_id) noexcept {drop_node_ = node_id;}
  [[nodiscard]] const std::vector<std::vector<CanFrame>> & batches() const noexcept
  {
    return batches_;
  }

private:
  std::array<std::uint32_t, kSwerveModuleCount> positions_{100U, 200U, 300U, 400U};
  std::deque<ReceivedCanFrame> pending_;
  std::vector<std::vector<CanFrame>> batches_;
  std::optional<std::uint8_t> drop_node_;
  bool open_{false};
};

TEST(CanopenEncoderTest, EncodesSyncAndSdoUploadRequests)
{
  const auto sync = make_canopen_sync();
  EXPECT_EQ(sync.id, 0x080U);
  EXPECT_EQ(sync.length, 0U);

  const auto upload = make_canopen_sdo_upload(3U, 0x6501U, 0U);
  EXPECT_EQ(upload.id, 0x603U);
  EXPECT_EQ(upload.data[0], 0x40U);
  EXPECT_EQ(upload.data[1], 0x01U);
  EXPECT_EQ(upload.data[2], 0x65U);
}

TEST(CanopenEncoderTest, DecodesExpeditedSdoWidthsAndRejectsAbort)
{
  EXPECT_EQ(decode_canopen_sdo_upload(sdo_reply(2U, 0x6501U, 0U, 65536U),
      2U, 0x6501U, 0U), 65536U);
  EXPECT_EQ(decode_canopen_sdo_upload(sdo_reply(2U, 0x6502U, 0U, 24U, 0x4BU),
      2U, 0x6502U, 0U), 24U);
  EXPECT_THROW(
    static_cast<void>(decode_canopen_sdo_upload(
        sdo_reply(2U, 0x6501U, 0U, 0x06020000U, 0x80U), 2U, 0x6501U, 0U)),
    CanopenEncoderError);
}

TEST(CanopenEncoderTest, ReadsHardwareIdentityInTwoFourNodeBatches)
{
  auto transport = std::make_unique<FakeEncoderTransport>();
  auto * observer = transport.get();
  CanopenEncoderClient client{EncoderCanopenConfig{{1U, 2U, 3U, 4U}, 2000},
    std::move(transport)};
  client.open();

  const auto hardware = client.read_hardware_info();

  for (const auto & info : hardware) {
    EXPECT_EQ(info.counts_per_revolution, 65536U);
    EXPECT_EQ(info.distinguishable_revolutions, 24U);
  }
  ASSERT_EQ(observer->batches().size(), 2U);
  EXPECT_EQ(observer->batches()[0].size(), kSwerveModuleCount);
  EXPECT_EQ(observer->batches()[1].size(), kSwerveModuleCount);
}

TEST(CanopenEncoderTest, SyncCycleRoutesOutOfOrderTpdoByNode)
{
  auto transport = std::make_unique<FakeEncoderTransport>();
  auto * observer = transport.get();
  CanopenEncoderClient client{EncoderCanopenConfig{{1U, 2U, 3U, 4U}, 2000},
    std::move(transport)};
  client.open();

  const auto cycle = client.sample(std::chrono::steady_clock::now() + 2ms);

  EXPECT_TRUE(cycle.complete());
  EXPECT_EQ(cycle.positions, (std::array<std::uint32_t, 4U>{100U, 200U, 300U, 400U}));
  ASSERT_EQ(observer->batches().size(), 1U);
  ASSERT_EQ(observer->batches().front().size(), 1U);
  EXPECT_EQ(observer->batches().front().front().id, 0x080U);
}

TEST(CanopenEncoderTest, MissingTpdoRemainsExplicitWithoutBlocking)
{
  auto transport = std::make_unique<FakeEncoderTransport>();
  auto * observer = transport.get();
  observer->set_drop_node(3U);
  CanopenEncoderClient client{EncoderCanopenConfig{{1U, 2U, 3U, 4U}, 2000},
    std::move(transport)};
  client.open();

  const auto cycle = client.sample(std::chrono::steady_clock::now() + 2ms);

  EXPECT_FALSE(cycle.complete());
  EXPECT_FALSE(cycle.received[2]);
  EXPECT_TRUE(cycle.received[0]);
  EXPECT_TRUE(cycle.received[1]);
  EXPECT_TRUE(cycle.received[3]);
}

TEST(CanopenEncoderTest, RejectsInvalidOrDuplicateNodeIdentifiers)
{
  EXPECT_THROW(
    static_cast<void>(CanopenEncoderClient(
        EncoderCanopenConfig{{1U, 2U, 2U, 4U}, 2000},
        std::make_unique<FakeEncoderTransport>())),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(make_canopen_sdo_upload(0U, 0x6501U, 0U)),
    std::invalid_argument);
}

}  // namespace
}  // namespace dm_swerve_driver
