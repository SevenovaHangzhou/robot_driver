// Exercises the real client against a scripted Modbus TCP server on the loopback interface.
#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>
#include <vector>

#include "infra_modbus/client.hpp"

namespace
{
using Bytes = std::vector<std::uint8_t>;
using namespace std::chrono_literals;

bool read_exactly(int fd, std::uint8_t * data, std::size_t size)
{
  std::size_t done = 0U;
  while (done < size) {
    const ssize_t count = ::recv(fd, data + done, size - done, 0);
    if (count <= 0) {
      return false;
    }
    done += static_cast<std::size_t>(count);
  }
  return true;
}

// Reads one request frame; returns MBAP header + PDU, or empty when the peer closed.
Bytes read_request(int fd)
{
  Bytes frame(7);
  if (!read_exactly(fd, frame.data(), frame.size())) {
    return {};
  }
  const std::size_t length = static_cast<std::size_t>((frame[4] << 8) | frame[5]);
  frame.resize(6U + length);
  if (!read_exactly(fd, frame.data() + 7, length - 1U)) {
    return {};
  }
  return frame;
}

Bytes reply_to(const Bytes & request, const Bytes & pdu)
{
  const auto length = static_cast<std::uint16_t>(pdu.size() + 1U);
  Bytes frame{
    request[0],
    request[1],
    0U,
    0U,
    static_cast<std::uint8_t>(length >> 8),
    static_cast<std::uint8_t>(length),
    request[6]};
  frame.insert(frame.end(), pdu.begin(), pdu.end());
  return frame;
}

void send_all(int fd, const Bytes & data)
{
  ASSERT_EQ(::send(fd, data.data(), data.size(), MSG_NOSIGNAL), static_cast<ssize_t>(data.size()));
}

// Each accepted connection is handed to `handler` with its 1-based connection number.
class Server
{
public:
  explicit Server(std::function<void(int fd, int connection)> handler)
  : handler_(std::move(handler))
  {
    listener_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    EXPECT_EQ(::bind(listener_, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
    socklen_t size = sizeof(address);
    ::getsockname(listener_, reinterpret_cast<sockaddr *>(&address), &size);
    port_ = ntohs(address.sin_port);
    ::listen(listener_, 4);
    thread_ = std::thread([this]() {
      for (int connection = 1;; ++connection) {
        const int fd = ::accept(listener_, nullptr, nullptr);
        if (fd < 0) {
          return;
        }
        accepted_ = connection;
        workers_.emplace_back([this, fd, connection]() {
          handler_(fd, connection);
          ::close(fd);
        });
      }
    });
  }
  ~Server()
  {
    ::shutdown(listener_, SHUT_RDWR);
    ::close(listener_);
    thread_.join();
    for (auto & worker : workers_) {
      worker.join();
    }
  }
  std::uint16_t port() const { return port_; }
  int accepted() const { return accepted_; }

private:
  std::function<void(int, int)> handler_;
  int listener_{-1};
  std::uint16_t port_{0};
  std::atomic<int> accepted_{0};
  std::thread thread_;
  std::vector<std::thread> workers_;  // only touched by thread_ until it is joined
};

const Bytes kReadCoil{0x01, 0x00, 0x05, 0x00, 0x01};

TEST(ClientTest, RejectsNonNumericHostAndInvalidSettings)
{
  EXPECT_THROW(infra_modbus::Client("localhost", 502, 1000ms), std::invalid_argument);
  EXPECT_THROW(infra_modbus::Client("127.0.0.1", 0, 1000ms), std::invalid_argument);
  EXPECT_THROW(infra_modbus::Client("127.0.0.1", 502, 0ms), std::invalid_argument);
  EXPECT_NO_THROW(infra_modbus::Client("::1", 502, 1000ms));
}

TEST(ClientTest, KeepsOneConnectionAcrossRequestsAndChecksTheReplyHeader)
{
  Server server([](int fd, int) {
    for (Bytes request; !(request = read_request(fd)).empty();) {
      EXPECT_EQ(request[6], 7U);
      // Reply byte by byte: TCP may split a frame anywhere.
      for (const auto byte : reply_to(request, {0x01, 0x01, 0x01})) {
        send_all(fd, {byte});
      }
    }
  });
  infra_modbus::Client client("127.0.0.1", server.port(), 1000ms);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(client.transact(7, kReadCoil), (Bytes{0x01, 0x01, 0x01}));
  }
  EXPECT_TRUE(client.connected());
  EXPECT_EQ(server.accepted(), 1);
}

TEST(ClientTest, ExceptionResponseKeepsTheConnection)
{
  Server server([](int fd, int) {
    Bytes request = read_request(fd);
    send_all(fd, reply_to(request, {0x81, 0x02}));
    request = read_request(fd);
    send_all(fd, reply_to(request, {0x01, 0x01, 0x00}));
    read_request(fd);
  });
  infra_modbus::Client client("127.0.0.1", server.port(), 1000ms);
  EXPECT_THROW(client.transact(1, kReadCoil), infra_modbus::ModbusException);
  EXPECT_TRUE(client.connected());
  EXPECT_EQ(client.transact(1, kReadCoil), (Bytes{0x01, 0x01, 0x00}));
  EXPECT_EQ(server.accepted(), 1);
}

TEST(ClientTest, TimeoutClosesSoALateReplyIsNeverReadAsTheNextResponse)
{
  Server server([](int fd, int connection) {
    Bytes request = read_request(fd);
    if (connection == 1) {
      std::this_thread::sleep_for(300ms);
      send_all(fd, reply_to(request, {0x01, 0x01, 0x01}));  // late
      read_request(fd);
      return;
    }
    send_all(fd, reply_to(request, {0x01, 0x01, 0x00}));
    read_request(fd);
  });
  infra_modbus::Client client("127.0.0.1", server.port(), 100ms);
  EXPECT_THROW(client.transact(1, kReadCoil), std::runtime_error);
  EXPECT_FALSE(client.connected());
  EXPECT_EQ(client.transact(1, kReadCoil), (Bytes{0x01, 0x01, 0x00}));
  EXPECT_EQ(server.accepted(), 2);
}

TEST(ClientTest, PerCallTimeoutOverridesTheDefault)
{
  Server server([](int fd, int) {
    Bytes request = read_request(fd);
    std::this_thread::sleep_for(200ms);
    send_all(fd, reply_to(request, {0x01, 0x01, 0x01}));
    read_request(fd);
  });
  infra_modbus::Client client("127.0.0.1", server.port(), 2000ms);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(client.transact(1, kReadCoil, 50ms), std::runtime_error);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1000ms);
  EXPECT_FALSE(client.connected());
  EXPECT_THROW(client.transact(1, kReadCoil, 0ms), std::invalid_argument);
}

TEST(ClientTest, MismatchedTransactionOrFunctionClosesTheConnection)
{
  Server server([](int fd, int connection) {
    Bytes request = read_request(fd);
    Bytes reply = connection == 1 ? reply_to(request, {0x01, 0x01, 0x01})
                                  : reply_to(request, {0x03, 0x02, 0x00, 0x00});
    if (connection == 1) {
      reply[1] = static_cast<std::uint8_t>(reply[1] + 1U);
    }
    send_all(fd, reply);
    read_request(fd);
  });
  infra_modbus::Client client("127.0.0.1", server.port(), 1000ms);
  EXPECT_THROW(client.transact(1, kReadCoil), std::runtime_error);
  EXPECT_FALSE(client.connected());
  EXPECT_THROW(client.transact(1, kReadCoil), std::runtime_error);
  EXPECT_FALSE(client.connected());
  EXPECT_EQ(server.accepted(), 2);
}

TEST(ClientTest, PeerCloseIsDetectedAndTheNextCallReconnects)
{
  Server server([](int fd, int connection) {
    Bytes request = read_request(fd);
    if (connection == 1) {
      return;  // close without replying
    }
    send_all(fd, reply_to(request, {0x01, 0x01, 0x01}));
    read_request(fd);
  });
  infra_modbus::Client client("127.0.0.1", server.port(), 1000ms);
  EXPECT_THROW(client.transact(1, kReadCoil), std::runtime_error);
  EXPECT_EQ(client.transact(1, kReadCoil), (Bytes{0x01, 0x01, 0x01}));
  EXPECT_EQ(server.accepted(), 2);
}
}  // namespace
