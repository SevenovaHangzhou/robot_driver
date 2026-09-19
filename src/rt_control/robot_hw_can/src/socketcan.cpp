#include "robot_hw_can/socketcan.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>

namespace robot_hw_can
{
namespace
{

[[noreturn]] void throw_system_error(const char * action)
{
  throw std::system_error{errno, std::generic_category(), action};
}

}  // namespace

SocketCan::~SocketCan() noexcept
{
  close();
}

void SocketCan::open(
  const std::string & interface_name, const std::vector<std::uint16_t> & receive_ids)
{
  if (interface_name.empty() || interface_name.size() >= IFNAMSIZ) {
    throw std::invalid_argument{"SocketCAN interface name is empty or too long"};
  }
  if (receive_ids.empty() ||
    std::any_of(receive_ids.begin(), receive_ids.end(), [](std::uint16_t id) {
      return id > CAN_SFF_MASK;
    }))
  {
    throw std::invalid_argument{"SocketCAN receive IDs must be non-empty standard IDs"};
  }

  const int pending = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (pending < 0) {
    throw_system_error("create SocketCAN socket");
  }

  try {
    std::vector<can_filter> filters;
    filters.reserve(receive_ids.size());
    for (const auto id : receive_ids) {
      filters.push_back(can_filter{
        id, static_cast<canid_t>(CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG)});
    }
    if (::setsockopt(
        pending, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
        static_cast<socklen_t>(filters.size() * sizeof(can_filter))) < 0)
    {
      throw_system_error("configure SocketCAN receive filters");
    }
    const int receive_own_messages{0};
    if (::setsockopt(
        pending, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
        &receive_own_messages, sizeof(receive_own_messages)) < 0)
    {
      throw_system_error("disable SocketCAN own-message reception");
    }

    ifreq request{};
    std::strncpy(request.ifr_name, interface_name.c_str(), sizeof(request.ifr_name) - 1U);
    if (::ioctl(pending, SIOCGIFINDEX, &request) < 0) {
      throw_system_error("resolve SocketCAN interface");
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(pending, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
      throw_system_error("bind SocketCAN interface");
    }
  } catch (...) {
    static_cast<void>(::close(pending));
    throw;
  }

  close();
  descriptor_ = pending;
}

void SocketCan::close() noexcept
{
  if (descriptor_ >= 0) {
    static_cast<void>(::close(descriptor_));
    descriptor_ = -1;
  }
}

bool SocketCan::is_open() const noexcept
{
  return descriptor_ >= 0;
}

void SocketCan::send(const CanFrame & frame) const
{
  if (!is_open()) {
    throw std::logic_error{"SocketCAN socket is not open"};
  }
  if (frame.id > CAN_SFF_MASK || frame.length != CAN_MAX_DLEN) {
    throw std::invalid_argument{"outgoing DaMiao frame must be an eight-byte standard frame"};
  }
  can_frame native{};
  native.can_id = frame.id;
  native.can_dlc = frame.length;
  std::copy(frame.data.begin(), frame.data.end(), native.data);
  const ssize_t sent = ::send(descriptor_, &native, sizeof(native), MSG_DONTWAIT | MSG_NOSIGNAL);
  if (sent < 0) {
    throw_system_error("send SocketCAN frame");
  }
  if (sent != static_cast<ssize_t>(sizeof(native))) {
    throw std::runtime_error{"partial SocketCAN frame write"};
  }
}

bool SocketCan::receive(CanFrame & frame) const
{
  if (!is_open()) {
    throw std::logic_error{"SocketCAN socket is not open"};
  }
  can_frame native{};
  const ssize_t received = ::recv(descriptor_, &native, sizeof(native), MSG_DONTWAIT);
  if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return false;
  }
  if (received < 0) {
    throw_system_error("receive SocketCAN frame");
  }
  if (received != static_cast<ssize_t>(sizeof(native)) ||
    (native.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U ||
    native.can_dlc != CAN_MAX_DLEN)
  {
    throw std::runtime_error{"received malformed DaMiao CAN frame"};
  }
  frame.id = static_cast<std::uint16_t>(native.can_id & CAN_SFF_MASK);
  frame.length = native.can_dlc;
  std::copy(native.data, native.data + CAN_MAX_DLEN, frame.data.begin());
  return true;
}

bool SocketCan::wait_readable(int timeout_ms) const
{
  pollfd descriptor{descriptor_, POLLIN, 0};
  int result{0};
  do {
    result = ::poll(&descriptor, 1U, timeout_ms);
  } while (result < 0 && errno == EINTR);
  if (result < 0) {
    throw_system_error("poll SocketCAN socket");
  }
  if (result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    throw std::runtime_error{"SocketCAN socket reported an I/O error"};
  }
  return result > 0 && (descriptor.revents & POLLIN) != 0;
}

}  // namespace robot_hw_can
