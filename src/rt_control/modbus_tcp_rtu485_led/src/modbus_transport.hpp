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
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace modbus_tcp_rtu485_led
{
using Deadline = std::chrono::steady_clock::time_point;

inline void validate_config(
  const std::string & ip, const std::vector<int64_t> & ports,
  const std::vector<int64_t> & addresses, int64_t timeout_ms)
{
  in_addr address{};
  if (inet_pton(AF_INET, ip.c_str(), &address) != 1) {
    throw std::invalid_argument("gateway_ip must be an IPv4 address");
  }
  if (ports.size() != 4 || addresses.size() != 4) {
    throw std::invalid_argument("gateway_ports and controller_addresses require four values");
  }
  for (size_t i = 0; i < 4; ++i) {
    if (ports[i] < 1 || ports[i] > 65535 || addresses[i] < 1 || addresses[i] > 247) {
      throw std::invalid_argument("ports must be 1..65535; controller addresses must be 1..247");
    }
  }
  if (timeout_ms < 1 || timeout_ms > 60000) {
    throw std::invalid_argument("response_timeout_ms must be 1..60000");
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

class Socket final
{
public:
  explicit Socket(int fd) : fd_(fd) {}
  ~Socket() {if (fd_ >= 0) {::close(fd_);}}
  Socket(const Socket &) = delete;
  Socket & operator=(const Socket &) = delete;
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
    if (item.revents & POLLNVAL) {throw std::runtime_error("Invalid Modbus socket");}
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

inline void exchange(int fd, std::vector<uint8_t> request, Deadline deadline)
{
  transfer(fd, request.data(), request.size(), true, deadline);
  std::array<uint8_t, 12> response{};
  transfer(fd, response.data(), 7, false, deadline);
  if (!std::equal(request.begin(), request.begin() + 4, response.begin()) ||
    response[4] != 0 || response[6] != request[6] ||
    (response[5] != 6 && response[5] != 3)) {
    throw std::runtime_error("Invalid Modbus response header");
  }
  transfer(fd, response.data() + 7, static_cast<size_t>(response[5] - 1), false, deadline);
  if (response[5] == 3 && response[7] == 0x90) {
    throw std::runtime_error("Modbus exception code " + std::to_string(response[8]));
  }
  if (response[5] != 6 ||
    !std::equal(request.begin() + 7, request.begin() + 12, response.begin() + 7)) {
    throw std::runtime_error("Invalid Modbus write acknowledgement");
  }
}

inline void send_color_tcp(
  const std::string & ip, uint16_t port, std::vector<uint8_t> request, int64_t timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
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
  exchange(socket.get(), std::move(request), deadline);
}
}  // namespace modbus_tcp_rtu485_led
