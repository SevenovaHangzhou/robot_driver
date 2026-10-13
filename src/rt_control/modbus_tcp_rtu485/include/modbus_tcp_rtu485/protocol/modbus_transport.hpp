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

void validate_endpoint(const std::string & ip, int64_t port, int64_t unit, int64_t timeout_ms);

void validate_led_config(
  const std::string & ip, const std::vector<int64_t> & ports,
  const std::vector<int64_t> & addresses, int64_t timeout_ms);

uint8_t brightness(float value);

inline constexpr std::array<uint8_t, 4> led_exit_color(bool abnormal)
{
  return abnormal ? std::array<uint8_t, 4>{255U, 0U, 0U, 0U}
                  : std::array<uint8_t, 4>{0U, 0U, 0U, 0U};
}

std::vector<uint8_t> color_request(
  uint16_t transaction, uint8_t unit, const std::array<uint8_t, 4> & values);

std::vector<uint8_t> read_holding_request(
  uint16_t transaction, uint8_t unit, uint16_t first_register, uint16_t count);

class Socket final
{
public:
  explicit Socket(int fd) : fd_(fd) {}
  ~Socket()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }
  Socket(const Socket &) = delete;
  Socket & operator=(const Socket &) = delete;
  Socket(Socket && other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
  Socket & operator=(Socket &&) = delete;
  int get() const { return fd_; }

private:
  int fd_;
};

void wait_ready(int fd, short events, Deadline deadline);

void transfer(int fd, uint8_t * data, size_t size, bool sending, Deadline deadline);

std::vector<uint8_t> exchange(int fd, const std::vector<uint8_t> & request, Deadline deadline);

Socket connect_tcp(const std::string & ip, uint16_t port, Deadline deadline);

std::vector<uint8_t> transact_tcp(
  const std::string & ip, uint16_t port, const std::vector<uint8_t> & request, int64_t timeout_ms);

void send_color_tcp(
  const std::string & ip, uint16_t port, const std::vector<uint8_t> & request, int64_t timeout_ms);

std::vector<uint16_t> parse_holding_response(const std::vector<uint8_t> & response, uint16_t count);

std::vector<uint16_t> read_holding_registers_tcp(
  const std::string & ip, uint16_t port, uint16_t transaction, uint8_t unit,
  uint16_t first_register, uint16_t count, int64_t timeout_ms);
}  // namespace modbus_tcp_rtu485
