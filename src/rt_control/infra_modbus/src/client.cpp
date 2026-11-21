#include "infra_modbus/client.hpp"

#include <netdb.h>
#include <net/if.h>
#include <poll.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <memory>
#include <thread>

namespace infra_modbus
{
namespace
{

std::uint16_t read_uint16_be(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>((data[0] << 8) | data[1]);
}

// Deadline expiry; lets exchange() tell a missing reply from other failures.
class Timeout : public std::runtime_error
{
public:
  Timeout() : std::runtime_error{"Modbus timeout"} {}
};

}  // namespace

Client::Client(
  const std::string & host, std::uint16_t port, std::chrono::milliseconds timeout,
  const std::string & interface)
: interface_(interface), timeout_(timeout)
{
  if (interface.size() >= IFNAMSIZ || interface.find('\0') != std::string::npos) {
    throw std::invalid_argument{"Invalid network interface name"};
  }
  if (port == 0U) {
    throw std::invalid_argument{"Modbus port must be 1..65535"};
  }
  if (timeout.count() <= 0) {
    throw std::invalid_argument{"Modbus timeout must be positive"};
  }
  addrinfo hints{};
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_NUMERICHOST | AI_NUMERICSERV;
  addrinfo * result = nullptr;
  const int rc = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &result);
  if (rc != 0) {
    throw std::invalid_argument{"Modbus host must be a numeric address: " + host};
  }
  const std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> owner(result, freeaddrinfo);
  std::memcpy(&address_, result->ai_addr, result->ai_addrlen);
  address_length_ = result->ai_addrlen;
}

Client::~Client() noexcept
{
  close();
}

void Client::close() noexcept
{
  if (descriptor_ >= 0) {
    static_cast<void>(::close(descriptor_));
    descriptor_ = -1;
  }
  consecutive_reply_timeouts_ = 0U;
}

bool Client::connected() const noexcept
{
  return descriptor_ >= 0;
}

void Client::set_min_request_gap(std::chrono::milliseconds gap)
{
  if (gap.count() < 0) {
    throw std::invalid_argument{"Modbus request gap must not be negative"};
  }
  min_request_gap_ = gap;
}

std::vector<std::uint8_t> Client::transact(std::uint8_t unit, const std::vector<std::uint8_t> & pdu)
{
  return transact(unit, pdu, timeout_);
}

std::vector<std::uint8_t> Client::transact(
  std::uint8_t unit, const std::vector<std::uint8_t> & pdu, std::chrono::milliseconds timeout)
{
  if (pdu.empty() || pdu.size() > 253U) {
    throw std::invalid_argument{"Modbus PDU must be 1..253 bytes"};
  }
  if (timeout.count() <= 0) {
    throw std::invalid_argument{"Modbus timeout must be positive"};
  }
  if (min_request_gap_.count() > 0) {
    const auto earliest = last_transaction_end_ + min_request_gap_;
    if (Clock::now() < earliest) {
      std::this_thread::sleep_until(earliest);
    }
  }
  struct MarkEnd
  {
    Clock::time_point & end;
    ~MarkEnd() { end = Clock::now(); }
  } mark_end{last_transaction_end_};
  return exchange(unit, pdu, Clock::now() + timeout);
}

std::vector<std::uint8_t> Client::exchange(
  std::uint8_t unit, const std::vector<std::uint8_t> & pdu, Clock::time_point deadline)
{
  // True only while the stream sits at a frame boundary after the request was fully sent.
  bool awaiting_reply = false;
  try {
    if (descriptor_ < 0) {
      connect(deadline);
    }
    ++transaction_id_;
    // MBAP header: transaction id, protocol id 0, remaining length, unit id; big-endian.
    const auto length = static_cast<std::uint16_t>(pdu.size() + 1U);
    std::vector<std::uint8_t> frame{
      static_cast<std::uint8_t>(transaction_id_ >> 8),
      static_cast<std::uint8_t>(transaction_id_),
      0U,
      0U,
      static_cast<std::uint8_t>(length >> 8),
      static_cast<std::uint8_t>(length),
      unit};
    frame.insert(frame.end(), pdu.begin(), pdu.end());
    transfer(frame.data(), frame.size(), true, deadline);

    std::array<std::uint8_t, 7> header{};
    std::vector<std::uint8_t> reply;
    for (;;) {
      awaiting_reply = true;
      wait(POLLIN, deadline);
      awaiting_reply = false;
      transfer(header.data(), header.size(), false, deadline);
      const auto reply_id = read_uint16_be(header.data());
      const auto reply_length = read_uint16_be(header.data() + 4);
      if (read_uint16_be(header.data() + 2) != 0U || reply_length < 2U || reply_length > 254U) {
        throw std::runtime_error{"invalid Modbus MBAP header"};
      }
      // The length field counts the unit id that is already read.
      reply.assign(reply_length - 1U, 0U);
      transfer(reply.data(), reply.size(), false, deadline);
      if (reply_id == transaction_id_) {
        break;
      }
      // Modulo-2^16 age of the reply id; a recent earlier id is a late reply to drop.
      const auto age = static_cast<std::uint16_t>(transaction_id_ - reply_id);
      if (age > kStaleTransactionWindow) {
        throw std::runtime_error{"invalid Modbus MBAP header"};
      }
    }
    if (header[6] != unit) {
      throw std::runtime_error{"invalid Modbus MBAP header"};
    }
    if (reply[0] == (pdu[0] | 0x80U)) {
      if (reply.size() != 2U) {
        throw std::runtime_error{"malformed Modbus exception response"};
      }
      consecutive_reply_timeouts_ = 0U;
      throw ModbusException{"Modbus exception code " + std::to_string(reply[1])};
    }
    if (reply[0] != pdu[0]) {
      throw std::runtime_error{"Modbus function code mismatch"};
    }
    consecutive_reply_timeouts_ = 0U;
    return reply;
  } catch (const ModbusException &) {
    throw;
  } catch (const Timeout &) {
    // No reply byte arrived: the stream is still aligned and a late reply will be discarded by
    // its transaction id, so keep the connection once. Reconnecting at once is refused by
    // single-client gateways, which would fail the next request to a healthy station.
    if (awaiting_reply && consecutive_reply_timeouts_ == 0U) {
      ++consecutive_reply_timeouts_;
      throw;
    }
    close();
    throw;
  } catch (...) {
    close();
    throw;
  }
}

void Client::connect(Clock::time_point deadline)
{
  descriptor_ = ::socket(address_.ss_family, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (descriptor_ < 0) {
    throw std::runtime_error{"cannot create Modbus socket"};
  }
  if (
    !interface_.empty() &&
    ::setsockopt(
      descriptor_, SOL_SOCKET, SO_BINDTODEVICE, interface_.c_str(), interface_.size() + 1) != 0) {
    throw std::runtime_error{
      "Modbus interface bind failed: " + interface_ + ": " + std::strerror(errno)};
  }
  if (::connect(descriptor_, reinterpret_cast<const sockaddr *>(&address_), address_length_) != 0) {
    if (errno != EINPROGRESS) {
      throw std::runtime_error{"Modbus connect failed"};
    }
    wait(POLLOUT, deadline);
    int error = 0;
    socklen_t size = sizeof(error);
    if (::getsockopt(descriptor_, SOL_SOCKET, SO_ERROR, &error, &size) != 0 || error != 0) {
      throw std::runtime_error{"Modbus connect failed"};
    }
  }
}

void Client::wait(short events, Clock::time_point deadline) const
{
  for (;;) {
    const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
    if (remaining <= 0) {
      throw Timeout{};
    }
    pollfd descriptor{descriptor_, events, 0};
    const int rc = ::poll(&descriptor, 1, static_cast<int>(remaining));
    if (rc < 0 && errno == EINTR) {
      continue;
    }
    if (rc == 0) {
      throw Timeout{};
    }
    if (rc < 0) {
      throw std::runtime_error{"Modbus poll failed"};
    }
    if ((descriptor.revents & events) != 0) {
      return;
    }
    throw std::runtime_error{"Modbus connection closed"};
  }
}

void Client::transfer(
  std::uint8_t * data, std::size_t size, bool sending, Clock::time_point deadline)
{
  std::size_t done = 0U;
  while (done < size) {
    wait(sending ? POLLOUT : POLLIN, deadline);
    const ssize_t count =
      sending ? ::send(descriptor_, data + done, size - done, MSG_NOSIGNAL | MSG_DONTWAIT)
              : ::recv(descriptor_, data + done, size - done, MSG_DONTWAIT);
    if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
      continue;
    }
    if (count <= 0) {
      throw std::runtime_error{"Modbus connection closed"};
    }
    done += static_cast<std::size_t>(count);
  }
}

}  // namespace infra_modbus
