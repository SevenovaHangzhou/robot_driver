#pragma once

// ROS-independent Modbus TCP client. Implementation: src/modbus_client.cpp.
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace plc_io_modbus
{
// 模块明确拒绝请求（如异常码 3），与断网/超时不同：完整回包后可继续使用连接。
class ModbusException : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// 同步客户端：一个请求完成后才处理下一个请求。每笔通讯共用 1 秒截止时间。
// 此类只能在非实时线程使用；TCP 分包时会继续收发，直到整帧完成或超时。
class ModbusClient
{
public:
  ~ModbusClient();

  void close();

  void connect(const std::string & host, int port, int unit);

  // value 在读请求中表示数量，在 FC05 写请求中表示 0xFF00/0x0000。
  std::vector<uint8_t> request(uint8_t function, uint16_t address, uint16_t value);
  // 将高、低两个字节合成无符号 16 位整数，例如 0x04、0xD2 -> 1234。
  static uint16_t read_uint16_be(const uint8_t * data);

private:
  using Clock = std::chrono::steady_clock;
  // 等待 socket 可读/可写；信号中断可重试，但不延长本次截止时间。
  void wait(short events);
  // TCP 不保证一次 recv/send 就得到整帧，循环累计已处理的字节数。
  void transfer_exactly(uint8_t * data, size_t size, bool writing);
  int socket_fd_{-1};
  uint8_t unit_id_{1};
  uint16_t transaction_id_{0};
  Clock::time_point deadline_;
};

}  // namespace plc_io_modbus
