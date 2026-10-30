#include <gtest/gtest.h>
#include <net/if.h>

#include <cerrno>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "infra_socketcan/socket.hpp"

namespace
{
const std::vector<can_filter> kAnyStandard{{0U, 0U}};

TEST(SocketTest, RejectsInvalidArguments)
{
  infra_socketcan::Socket socket;
  EXPECT_THROW(socket.open("", kAnyStandard), std::invalid_argument);
  EXPECT_THROW(socket.open("interface-name-too-long", kAnyStandard), std::invalid_argument);
  EXPECT_THROW(socket.open("vcan0", {}), std::invalid_argument);
  EXPECT_FALSE(socket.is_open());
}

TEST(SocketTest, MissingInterfaceThrowsAndLeavesSocketClosed)
{
  infra_socketcan::Socket socket;
  EXPECT_THROW(socket.open("nocan_test0", kAnyStandard), std::system_error);
  EXPECT_FALSE(socket.is_open());
  EXPECT_EQ(socket.interface_index(), 0);
}

TEST(SocketTest, ClosedSocketReportsErrorsWithoutThrowing)
{
  const infra_socketcan::Socket socket;
  can_frame frame{};
  EXPECT_EQ(socket.send(frame), -EBADF);
  EXPECT_EQ(socket.receive(frame), -EBADF);
  EXPECT_EQ(socket.wait_readable(0), 0);  // poll ignores a negative descriptor
}

// Real send/receive needs a virtual CAN interface (root: ip link add vcan0 type vcan).
TEST(SocketTest, VirtualInterfaceRoundTripExcludesOwnFrames)
{
  if (if_nametoindex("vcan0") == 0U) {
    GTEST_SKIP() << "vcan0 is not available";
  }
  infra_socketcan::Socket sender;
  infra_socketcan::Socket receiver;
  sender.open("vcan0", kAnyStandard);
  receiver.open("vcan0", kAnyStandard);
  EXPECT_EQ(receiver.interface_index(), static_cast<int>(if_nametoindex("vcan0")));

  can_frame sent{};
  sent.can_id = 0x123U;
  sent.can_dlc = 8U;
  sent.data[0] = 0x5AU;
  ASSERT_EQ(sender.send(sent), 0);
  ASSERT_EQ(receiver.wait_readable(1000), 1);
  can_frame received{};
  ASSERT_EQ(receiver.receive(received), 1);
  EXPECT_EQ(received.can_id, sent.can_id);
  EXPECT_EQ(received.data[0], 0x5AU);
  EXPECT_EQ(sender.receive(received), 0);

  infra_socketcan::Socket moved{std::move(receiver)};
  EXPECT_TRUE(moved.is_open());
  EXPECT_FALSE(receiver.is_open());  // NOLINT(bugprone-use-after-move)
}
}  // namespace
