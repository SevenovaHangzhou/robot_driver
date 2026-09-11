#pragma once

// Modbus TCP 通讯细节集中在此文件。ROS 话题和服务逻辑请阅读 node.cpp。
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
  ~ModbusClient()
  {
    close();
  }

  void close()
  {
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  void connect(const std::string & host, int port, int unit)
  {
    if (socket_fd_ >= 0) {
      return;
    }
    addrinfo hints{};
    hints.ai_socktype = SOCK_STREAM;
    addrinfo * result = nullptr;
    const auto service = std::to_string(port);
    const int rc = getaddrinfo(host.c_str(), service.c_str(), &hints, &result);
    if (rc != 0) {
      throw std::runtime_error(gai_strerror(rc));
    }
    std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> addresses(result, freeaddrinfo);
    unit_id_ = static_cast<uint8_t>(unit);
    for (auto * address = result; address; address = address->ai_next) {
      socket_fd_ = socket(address->ai_family, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
      if (socket_fd_ < 0) {
        continue;
      }
      try {
        deadline_ = Clock::now() + std::chrono::seconds(1);
        if (::connect(socket_fd_, address->ai_addr, address->ai_addrlen) != 0) {
          if (errno != EINPROGRESS) {
            throw std::runtime_error("connect failed");
          }
          wait(POLLOUT);
          int error = 0;
          socklen_t size = sizeof(error);
          if (getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &error, &size) != 0 || error != 0) {
            throw std::runtime_error("connect failed");
          }
        }
        return;
      } catch (...) {
        // 当前候选地址连接失败，关闭后尝试下一个解析结果。
        close();
      }
    }
    throw std::runtime_error("IO module connection failed");
  }

  // value 在读请求中表示数量，在 FC05 写请求中表示 0xFF00/0x0000。
  std::vector<uint8_t> request(uint8_t function, uint16_t address, uint16_t value)
  {
    if (socket_fd_ < 0) {
      throw std::runtime_error("IO module not connected");
    }
    deadline_ = Clock::now() + std::chrono::seconds(1);
    ++transaction_id_;
    // MBAP 头：事务编号(2字节)、协议号(2)、后续长度(2)、单元标识(1)。
    // 请求正文：功能码(1)、地址(2)、数量或写入值(2)。多字节数值高字节在前。
    std::array<uint8_t, 12> frame{
      static_cast<uint8_t>(transaction_id_ >> 8), static_cast<uint8_t>(transaction_id_),
      0, 0, 0, 6, unit_id_, function,
      static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address),
      static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
    try {
      transfer_exactly(frame.data(), frame.size(), true);
      std::array<uint8_t, 7> header{};
      transfer_exactly(header.data(), header.size(), false);
      const auto reply_transaction_id = read_uint16_be(header.data());
      const auto protocol_id = read_uint16_be(header.data() + 2);
      const auto length = read_uint16_be(header.data() + 4);
      const auto reply_unit_id = header[6];
      if (reply_transaction_id != transaction_id_ || protocol_id != 0 ||
        reply_unit_id != unit_id_ || length < 2 || length > 254)
      {
        throw std::runtime_error("invalid Modbus MBAP header");
      }
      // length 包括已经读过的单元标识，因此正文还剩 length - 1 字节。
      // pdu[0] 是功能码；读取正常响应的 pdu[1] 是数据字节数。
      std::vector<uint8_t> pdu(length - 1);
      transfer_exactly(pdu.data(), pdu.size(), false);
      if (pdu[0] == (function | 0x80)) {
        if (pdu.size() != 2) {
          throw std::runtime_error("malformed Modbus exception");
        }
        throw ModbusException("Modbus exception: " + std::to_string(pdu[1]));
      }
      if (pdu[0] != function) {
        throw std::runtime_error("Modbus function mismatch");
      }
      return pdu;
    } catch (const ModbusException & error) {
      throw ModbusException("FC=" + std::to_string(function) +
        " address=" + std::to_string(address) + " value/count=" + std::to_string(value) +
        ": " + error.what());
    } catch (const std::exception & error) {
      close();
      throw std::runtime_error("FC=" + std::to_string(function) +
        " address=" + std::to_string(address) + " value/count=" + std::to_string(value) +
        ": " + error.what());
    }
  }
  // 将高、低两个字节合成无符号 16 位整数，例如 0x04、0xD2 -> 1234。
  static uint16_t read_uint16_be(const uint8_t * data)
  {
    return static_cast<uint16_t>((data[0] << 8) | data[1]);
  }

private:
  using Clock = std::chrono::steady_clock;
  // 等待 socket 可读/可写；信号中断可重试，但不延长本次截止时间。
  void wait(short events)
  {
    for (;;) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline_ - Clock::now()).count();
      if (remaining <= 0) {
        throw std::runtime_error("Modbus timeout");
      }
      pollfd descriptor{socket_fd_, events, 0};
      const int rc = ::poll(&descriptor, 1, static_cast<int>(remaining));
      if (rc < 0 && errno == EINTR) {
        continue;
      }
      if (rc <= 0) {
        throw std::runtime_error("Modbus timeout or socket error");
      }
      if (descriptor.revents & events) {
        return;
      }
      throw std::runtime_error("Modbus connection closed");
    }
  }
  // TCP 不保证一次 recv/send 就得到整帧，循环累计已处理的字节数。
  void transfer_exactly(uint8_t * data, size_t size, bool writing)
  {
    size_t done = 0;
    while (done < size) {
      wait(writing ? POLLOUT : POLLIN);
      ssize_t count = 0;
      if (writing) {
        count = ::send(socket_fd_, data + done, size - done, MSG_NOSIGNAL);
      } else {
        count = ::recv(socket_fd_, data + done, size - done, 0);
      }
      if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
        continue;
      }
      if (count <= 0) {
        throw std::runtime_error("Modbus connection closed");
      }
      done += static_cast<size_t>(count);
    }
  }
  int socket_fd_{-1};
  uint8_t unit_id_{1};
  uint16_t transaction_id_{0};
  Clock::time_point deadline_;
};

}  // namespace plc_io_modbus
