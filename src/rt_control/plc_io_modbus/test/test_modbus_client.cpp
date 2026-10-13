#include "plc_io_modbus/protocol/modbus_client.hpp"

#include <gtest/gtest.h>

#include <cstring>

namespace
{
std::vector<uint8_t> incoming;
std::vector<uint8_t> outgoing;
size_t read_offset;
int closes;
int connections;
bool poll_timeout;
sockaddr fake_address{};
addrinfo fake_info{};

class ModbusClientTest : public testing::Test
{
protected:
  void SetUp() override
  {
    incoming.clear();
    outgoing.clear();
    read_offset = 0;
    closes = 0;
    connections = 0;
    poll_timeout = false;
  }
  void reply(std::vector<uint8_t> bytes)
  {
    incoming = std::move(bytes);
    read_offset = 0;
    outgoing.clear();
  }
};
}  // namespace

extern "C" int __wrap_getaddrinfo(const char *, const char *, const addrinfo *, addrinfo ** result)
{
  fake_info = {};
  fake_info.ai_family = AF_INET;
  fake_info.ai_addr = &fake_address;
  fake_info.ai_addrlen = sizeof(fake_address);
  *result = &fake_info;
  return 0;
}
extern "C" void __wrap_freeaddrinfo(addrinfo *) {}
extern "C" int __wrap_socket(int, int, int)
{
  return 123;
}
extern "C" int __wrap_connect(int, const sockaddr *, socklen_t)
{
  ++connections;
  return 0;
}
extern "C" int __wrap_close(int)
{
  ++closes;
  return 0;
}
extern "C" int __wrap_poll(pollfd * descriptors, nfds_t, int)
{
  descriptors[0].revents = descriptors[0].events;
  return poll_timeout ? 0 : 1;
}
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

TEST_F(ModbusClientTest, FragmentedReadAndCoilEchoPreserveFramesAndCloseOnce)
{
  {
    plc_io_modbus::ModbusClient client;
    EXPECT_THROW(client.request(4, 0, 1), std::runtime_error);
    client.connect("fake", 502, 1);
    client.connect("fake", 502, 1);
    EXPECT_EQ(connections, 1);
    // Existing read_uint16_be example (0x04D2 = 1234) and node FC04/FC05 paths.
    reply({0, 1, 0, 0, 0, 5, 1, 4, 2, 4, 0xD2});
    const auto response = client.request(4, 0, 1);
    EXPECT_EQ(response, (std::vector<uint8_t>{4, 2, 4, 0xD2}));
    EXPECT_EQ(plc_io_modbus::ModbusClient::read_uint16_be(response.data() + 2), 1234);
    EXPECT_EQ(outgoing, (std::vector<uint8_t>{0, 1, 0, 0, 0, 6, 1, 4, 0, 0, 0, 1}));
    reply({0, 2, 0, 0, 0, 6, 1, 5, 0, 0, 0xFF, 0});
    EXPECT_EQ(client.request(5, 0, 0xFF00), (std::vector<uint8_t>{5, 0, 0, 0xFF, 0}));
    EXPECT_EQ(outgoing, (std::vector<uint8_t>{0, 2, 0, 0, 0, 6, 1, 5, 0, 0, 0xFF, 0}));
    client.close();
    client.close();
  }
  EXPECT_EQ(closes, 1);
}

TEST_F(ModbusClientTest, DeviceExceptionKeepsConnectionAndErrorContext)
{
  plc_io_modbus::ModbusClient client;
  client.connect("fake", 502, 1);
  reply({0, 1, 0, 0, 0, 3, 1, 0x84, 3});
  try {
    client.request(4, 0, 1);
    FAIL() << "Expected device exception";
  } catch (const plc_io_modbus::ModbusException & error) {
    EXPECT_STREQ(error.what(), "FC=4 address=0 value/count=1: Modbus exception: 3");
  }
  EXPECT_EQ(closes, 0);
  reply({0, 2, 0, 0, 0, 5, 1, 4, 2, 4, 0xD2});
  EXPECT_NO_THROW(client.request(4, 0, 1));
  EXPECT_EQ(connections, 1);
}

TEST_F(ModbusClientTest, MalformedOrInterruptedReplyClosesUntilReconnect)
{
  const std::vector<uint8_t> response{0, 1, 0, 0, 0, 5, 1, 4, 2, 4, 0xD2};
  for (const size_t offset : {size_t{0}, size_t{2}, size_t{4}, size_t{6}, size_t{7}}) {
    plc_io_modbus::ModbusClient client;
    client.connect("fake", 502, 1);
    auto invalid = response;
    invalid[offset] ^= 1;
    reply(invalid);
    EXPECT_THROW(client.request(4, 0, 1), std::runtime_error);
    EXPECT_THROW(client.request(4, 0, 1), std::runtime_error);
    client.connect("fake", 502, 1);
    reply({0, 2, 0, 0, 0, 5, 1, 4, 2, 4, 0xD2});
    EXPECT_NO_THROW(client.request(4, 0, 1));
  }
  EXPECT_EQ(closes, 10);
  for (const bool timeout : {false, true}) {
    plc_io_modbus::ModbusClient client;
    client.connect("fake", 502, 1);
    reply({});
    poll_timeout = timeout;
    EXPECT_THROW(client.request(4, 0, 1), std::runtime_error);
    EXPECT_THROW(client.request(4, 0, 1), std::runtime_error);
  }
  EXPECT_EQ(closes, 12);
}
