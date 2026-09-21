#pragma once

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace modbus_tcp_rtu485
{
using Deadline = std::chrono::steady_clock::time_point;
inline constexpr size_t kLedControllerCount{6U};

inline void validate_endpoint(
  const std::string & ip, int64_t port, int64_t unit, int64_t timeout_ms)
{
  in_addr address{};
  if (inet_pton(AF_INET, ip.c_str(), &address) != 1) {
    throw std::invalid_argument("gateway_ip must be an IPv4 address");
  }
  if (port < 1 || port > 65535) {
    throw std::invalid_argument("gateway port must be 1..65535");
  }
  if (unit < 1 || unit > 247) {
    throw std::invalid_argument("Modbus unit address must be 1..247");
  }
  if (timeout_ms < 1 || timeout_ms > 60000) {
    throw std::invalid_argument("response_timeout_ms must be 1..60000");
  }
}

inline void validate_led_config(
  const std::string & ip, const std::vector<int64_t> & ports,
  const std::vector<int64_t> & addresses, int64_t timeout_ms)
{
  if (ports.size() != kLedControllerCount || addresses.size() != kLedControllerCount) {
    throw std::invalid_argument("gateway_ports and controller_addresses require six values");
  }
  for (size_t i = 0; i < kLedControllerCount; ++i) {
    validate_endpoint(ip, ports[i], addresses[i], timeout_ms);
  }
}

inline uint8_t brightness(float value)
{
  if (!std::isfinite(value)) {return 0;}
  return static_cast<uint8_t>(std::lround(std::clamp(value, 0.0F, 1.0F) * 255.0F));
}

inline std::vector<uint8_t> color_request(
  uint16_t transaction, uint8_t unit, const std::array<uint8_t, 4> & values)
{
  std::vector<uint8_t> request{
    static_cast<uint8_t>(transaction >> 8U), static_cast<uint8_t>(transaction & 0xFFU),
    0, 0, 0, 15, unit, 0x10, 0, 0, 0, 4, 8};
  for (const auto value : values) {request.insert(request.end(), {0, value});}
  return request;
}

inline std::vector<uint8_t> read_holding_request(
  uint16_t transaction, uint8_t unit, uint16_t first_register, uint16_t count)
{
  if (count == 0 || count > 125) {
    throw std::invalid_argument("holding register count must be 1..125");
  }
  return {
    static_cast<uint8_t>(transaction >> 8U), static_cast<uint8_t>(transaction & 0xFFU),
    0, 0, 0, 6, unit, 0x03,
    static_cast<uint8_t>(first_register >> 8U),
    static_cast<uint8_t>(first_register & 0xFFU),
    static_cast<uint8_t>(count >> 8U), static_cast<uint8_t>(count & 0xFFU)};
}

class Socket final
{
public:
  explicit Socket(int fd) : fd_(fd) {}
  ~Socket() {if (fd_ >= 0) {::close(fd_);}}
  Socket(const Socket &) = delete;
  Socket & operator=(const Socket &) = delete;
  Socket(Socket && other) noexcept : fd_(other.fd_) {other.fd_ = -1;}
  Socket & operator=(Socket &&) = delete;
  int get() const {return fd_;}

private:
  int fd_;
};

inline void wait_ready(int fd, short events, Deadline deadline)
{
  while (true) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (remaining <= 0) {throw std::runtime_error("Modbus transaction timed out");}
    pollfd item{fd, events, 0};
    const int result = ::poll(&item, 1, static_cast<int>(remaining));
    if (result < 0 && errno == EINTR) {continue;}
    if (result <= 0) {throw std::runtime_error("Modbus poll failed or timed out");}
    if ((item.revents & POLLNVAL) != 0) {throw std::runtime_error("Invalid Modbus socket");}
    return;
  }
}

inline void transfer(int fd, uint8_t * data, size_t size, bool sending, Deadline deadline)
{
  size_t offset = 0;
  while (offset < size) {
    wait_ready(fd, sending ? POLLOUT : POLLIN, deadline);
    const auto count = sending ?
      ::send(fd, data + offset, size - offset, MSG_NOSIGNAL | MSG_DONTWAIT) :
      ::recv(fd, data + offset, size - offset, MSG_DONTWAIT);
    if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {continue;}
    if (count <= 0) {throw std::runtime_error("Modbus peer disconnected or IO failed");}
    offset += static_cast<size_t>(count);
  }
}

inline std::vector<uint8_t> exchange(
  int fd, const std::vector<uint8_t> & request, Deadline deadline)
{
  if (request.size() < 8) {throw std::invalid_argument("Modbus request is too short");}
  auto mutable_request = request;
  transfer(fd, mutable_request.data(), mutable_request.size(), true, deadline);

  std::vector<uint8_t> response(7);
  transfer(fd, response.data(), response.size(), false, deadline);
  const uint16_t length = static_cast<uint16_t>(
    (static_cast<uint16_t>(response[4]) << 8U) | response[5]);
  if (!std::equal(request.begin(), request.begin() + 4, response.begin()) ||
    response[6] != request[6] || length < 2 || length > 254) {
    throw std::runtime_error("Invalid Modbus response header");
  }
  response.resize(6U + length);
  transfer(fd, response.data() + 7, static_cast<size_t>(length - 1U), false, deadline);

  if (response[7] == static_cast<uint8_t>(request[7] | 0x80U)) {
    if (response.size() != 9) {throw std::runtime_error("Invalid Modbus exception response");}
    throw std::runtime_error("Modbus exception code " + std::to_string(response[8]));
  }
  if (response[7] != request[7]) {throw std::runtime_error("Unexpected Modbus function code");}
  return response;
}

inline Socket connect_tcp(const std::string & ip, uint16_t port, Deadline deadline)
{
  Socket socket(::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
  if (socket.get() < 0) {throw std::runtime_error("Cannot create Modbus socket");}
  sockaddr_in server{};
  server.sin_family = AF_INET;
  server.sin_port = htons(port);
  if (inet_pton(AF_INET, ip.c_str(), &server.sin_addr) != 1) {
    throw std::runtime_error("Invalid gateway IPv4 address");
  }
  if (::connect(socket.get(), reinterpret_cast<sockaddr *>(&server), sizeof(server)) < 0) {
    if (errno != EINPROGRESS) {throw std::runtime_error("Modbus connect failed");}
    wait_ready(socket.get(), POLLOUT, deadline);
    int error = 0;
    socklen_t length = sizeof(error);
    if (::getsockopt(socket.get(), SOL_SOCKET, SO_ERROR, &error, &length) < 0 || error != 0) {
      throw std::runtime_error("Modbus connect failed");
    }
  }
  return socket;
}

inline std::vector<uint8_t> transact_tcp(
  const std::string & ip, uint16_t port, const std::vector<uint8_t> & request,
  int64_t timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  auto socket = connect_tcp(ip, port, deadline);
  return exchange(socket.get(), request, deadline);
}

inline void send_color_tcp(
  const std::string & ip, uint16_t port, const std::vector<uint8_t> & request,
  int64_t timeout_ms)
{
  const auto response = transact_tcp(ip, port, request, timeout_ms);
  if (response.size() != 12 || request.size() < 12 ||
    !std::equal(request.begin() + 8, request.begin() + 12, response.begin() + 8)) {
    throw std::runtime_error("Invalid Modbus write acknowledgement");
  }
}

inline std::vector<uint16_t> parse_holding_response(
  const std::vector<uint8_t> & response, uint16_t count)
{
  const size_t byte_count = static_cast<size_t>(count) * 2U;
  if (response.size() != 9U + byte_count || response[7] != 0x03 ||
    response[8] != static_cast<uint8_t>(byte_count)) {
    throw std::runtime_error("Invalid Modbus holding register response");
  }
  std::vector<uint16_t> values;
  values.reserve(count);
  for (size_t offset = 9; offset < response.size(); offset += 2) {
    values.push_back(static_cast<uint16_t>(
      (static_cast<uint16_t>(response[offset]) << 8U) | response[offset + 1]));
  }
  return values;
}

inline std::vector<uint16_t> read_holding_registers_tcp(
  const std::string & ip, uint16_t port, uint16_t transaction, uint8_t unit,
  uint16_t first_register, uint16_t count, int64_t timeout_ms)
{
  const auto request = read_holding_request(transaction, unit, first_register, count);
  const auto response = transact_tcp(ip, port, request, timeout_ms);
  return parse_holding_response(response, count);
}
}  // namespace modbus_tcp_rtu485
