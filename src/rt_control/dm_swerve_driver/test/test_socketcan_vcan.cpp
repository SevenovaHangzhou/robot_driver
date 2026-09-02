#include <gtest/gtest.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "dm_swerve_driver/socketcan_interface.hpp"

namespace dm_swerve_driver {
namespace {

using namespace std::chrono_literals;

TEST(SocketCanInterfaceTest, RejectsInvalidConfiguration)
{
  EXPECT_THROW(
    SocketCanInterface(SocketCanOptions{"", {1U}, 2ms, false}),
    std::invalid_argument);
  EXPECT_THROW(
    SocketCanInterface(SocketCanOptions{"can_missing0", {1U}, 2ms, false}),
    SocketCanError);
  EXPECT_THROW(
    SocketCanInterface(SocketCanOptions{"vcan0", {0x800U}, 2ms, false}),
    std::invalid_argument);
}

TEST(SocketCanInterfaceTest, ClosedSocketRejectsIoAndEmptyBatchIsNoOp)
{
  SocketCanInterface socket;
  EXPECT_FALSE(socket.is_open());
  EXPECT_EQ(socket.native_handle(), -1);
  EXPECT_NO_THROW(socket.write_batch({}));
  EXPECT_THROW(socket.write_batch({CanFrame{}}), SocketCanError);
  EXPECT_THROW(socket.collect(1U, std::chrono::steady_clock::now()), SocketCanError);
}

TEST(SocketCanInterfaceTest, MovesFileDescriptorOwnership)
{
  SocketCanInterface empty;
  SocketCanInterface moved{std::move(empty)};
  EXPECT_FALSE(empty.is_open());
  EXPECT_FALSE(moved.is_open());
}

TEST(SocketCanVcanIntegrationTest, BatchFramesAreBackToBackAndRepliesAreCollected)
{
  SocketCanInterface driver;
  SocketCanInterface peer;
  try {
    driver.open(SocketCanOptions{"vcan0", {0x301U, 0x302U}, 5ms, false});
    peer.open(SocketCanOptions{
        "vcan0", {0x101U, 0x102U, 0x103U, 0x104U, 0x105U, 0x106U, 0x107U, 0x108U},
        5ms, false});
  } catch (const SocketCanError & error) {
    if (error.error_number() == ENODEV || error.error_number() == ENXIO ||
      error.error_number() == EAFNOSUPPORT || error.error_number() == EPROTONOSUPPORT)
    {
      GTEST_SKIP() << "vcan0 unavailable: " << error.what();
    }
    throw;
  }

  std::vector<CanFrame> commands;
  for (std::uint16_t id = 0x101U; id <= 0x108U; ++id) {
    CanFrame frame{};
    frame.id = id;
    frame.data[0] = static_cast<std::uint8_t>(id - 0x100U);
    commands.push_back(frame);
  }
  driver.write_batch(commands);

  const auto received = peer.collect(8U, std::chrono::steady_clock::now() + 20ms);
  ASSERT_EQ(received.size(), 8U);
  EXPECT_GT(received.front().kernel_timestamp.count(), 0);
  EXPECT_LT(
    received.back().kernel_timestamp - received.front().kernel_timestamp,
    1ms);

  CanFrame reply_one{};
  reply_one.id = 0x301U;
  CanFrame reply_two{};
  reply_two.id = 0x302U;
  peer.write_batch({reply_one, reply_two});
  EXPECT_EQ(driver.collect(2U, std::chrono::steady_clock::now() + 20ms).size(), 2U);
}

}  // namespace
}  // namespace dm_swerve_driver
