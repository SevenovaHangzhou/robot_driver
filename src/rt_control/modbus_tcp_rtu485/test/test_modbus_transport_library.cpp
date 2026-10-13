#include "modbus_tcp_rtu485/protocol/modbus_transport.hpp"

#include <gtest/gtest.h>

#include <cstring>

namespace
{
std::vector<uint8_t> incoming;
std::vector<uint8_t> outgoing;
size_t read_offset;
bool poll_timeout;

class ModbusTransportLibrary : public testing::Test
{
protected:
  void SetUp() override
  {
    incoming.clear();
    outgoing.clear();
    read_offset = 0;
    poll_timeout = false;
  }
  std::vector<uint8_t> run(const std::vector<uint8_t> & response)
  {
    incoming = response;
    read_offset = 0;
    outgoing.clear();
    return modbus_tcp_rtu485::exchange(
      123, modbus_tcp_rtu485::read_holding_request(1, 1, 0x0106, 4),
      std::chrono::steady_clock::now() + std::chrono::hours(1));
  }
};

// Vectors are retained from test_modbus_transport.cpp.
const std::vector<uint8_t> kReadResponse{0, 1,    0, 0,    0, 11,   1,    3,   8,
                                         0, 0x19, 0, 0x47, 0, 0x13, 0xFF, 0xFD};
}  // namespace

// Linker-injected IO: each call transfers one byte, without opening a socket.
extern "C" ssize_t __wrap_send(int, const void * data, size_t size, int)
{
  if (size == 0) {
    return 0;
  }
  outgoing.push_back(*static_cast<const uint8_t *>(data));
  return 1;
}

extern "C" ssize_t __wrap_recv(int, void * data, size_t size, int)
{
  if (size == 0 || read_offset == incoming.size()) {
    return 0;
  }
  std::memcpy(data, incoming.data() + read_offset++, 1);
  return 1;
}

extern "C" int __wrap_poll(pollfd * descriptors, nfds_t, int)
{
  descriptors[0].revents = descriptors[0].events;
  return poll_timeout ? 0 : 1;
}

TEST_F(ModbusTransportLibrary, FragmentedReadPreservesRequestAndDecodedRegisters)
{
  const auto reply = run(kReadResponse);
  EXPECT_EQ(outgoing, (std::vector<uint8_t>{0, 1, 0, 0, 0, 6, 1, 3, 1, 6, 0, 4}));
  EXPECT_EQ(reply, kReadResponse);
  EXPECT_EQ(
    modbus_tcp_rtu485::parse_holding_response(reply, 4),
    (std::vector<uint16_t>{0x0019, 0x0047, 0x0013, 0xFFFD}));
}

TEST_F(ModbusTransportLibrary, RejectsInvalidRepliesAndAllowsNextTransaction)
{
  for (const size_t offset : {size_t{0}, size_t{2}, size_t{4}, size_t{6}, size_t{7}}) {
    auto invalid = kReadResponse;
    invalid[offset] ^= 1;
    EXPECT_THROW(run(invalid), std::runtime_error);
  }
  EXPECT_THROW(run({0, 1, 0, 0, 0, 3, 1, 0x83, 2}), std::runtime_error);
  EXPECT_THROW(run({}), std::runtime_error);
  poll_timeout = true;
  EXPECT_THROW(run(kReadResponse), std::runtime_error);
  poll_timeout = false;
  EXPECT_EQ(run(kReadResponse), kReadResponse);
}

TEST_F(ModbusTransportLibrary, FragmentedWritePreservesRequestAndAcknowledgement)
{
  using namespace modbus_tcp_rtu485;
  const auto request = color_request(0x1234, 1, {255, 0, 128, 0});
  incoming = {0x12, 0x34, 0, 0, 0, 6, 1, 0x10, 0, 0, 0, 4};
  EXPECT_EQ(
    exchange(123, request, std::chrono::steady_clock::now() + std::chrono::hours(1)), incoming);
  EXPECT_EQ(outgoing, (std::vector<uint8_t>{0x12, 0x34, 0, 0,   0, 15, 1, 0x10, 0, 0, 0,
                                            4,    8,    0, 255, 0, 0,  0, 128,  0, 0}));
}
