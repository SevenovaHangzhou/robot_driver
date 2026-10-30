#include "infra_socketcan/socket.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace infra_socketcan
{
namespace
{

[[noreturn]] void throw_system_error(const char * action)
{
  throw std::system_error{errno, std::generic_category(), action};
}

}  // namespace

Socket::~Socket() noexcept
{
  close();
}

Socket::Socket(Socket && other) noexcept
: descriptor_(std::exchange(other.descriptor_, -1)),
  interface_index_(std::exchange(other.interface_index_, 0))
{
}

Socket & Socket::operator=(Socket && other) noexcept
{
  if (this != &other) {
    close();
    descriptor_ = std::exchange(other.descriptor_, -1);
    interface_index_ = std::exchange(other.interface_index_, 0);
  }
  return *this;
}

void Socket::open(const std::string & interface_name, const std::vector<can_filter> & filters)
{
  if (interface_name.empty() || interface_name.size() >= IFNAMSIZ) {
    throw std::invalid_argument{"SocketCAN interface name is empty or too long"};
  }
  if (filters.empty()) {
    throw std::invalid_argument{"SocketCAN receive filters must not be empty"};
  }

  const int pending = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (pending < 0) {
    throw_system_error("create SocketCAN socket");
  }

  ifreq request{};
  try {
    if (
      ::setsockopt(
        pending, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
        static_cast<socklen_t>(filters.size() * sizeof(can_filter))) < 0) {
      throw_system_error("configure SocketCAN receive filters");
    }
    const int receive_own_messages{0};
    if (
      ::setsockopt(
        pending, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receive_own_messages,
        sizeof(receive_own_messages)) < 0) {
      throw_system_error("disable SocketCAN own-message reception");
    }

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
  interface_index_ = request.ifr_ifindex;
}

void Socket::close() noexcept
{
  if (descriptor_ >= 0) {
    static_cast<void>(::close(descriptor_));
    descriptor_ = -1;
    interface_index_ = 0;
  }
}

bool Socket::is_open() const noexcept
{
  return descriptor_ >= 0;
}

int Socket::interface_index() const noexcept
{
  return interface_index_;
}

int Socket::send(const can_frame & frame) const noexcept
{
  const ssize_t sent = ::send(descriptor_, &frame, sizeof(frame), MSG_DONTWAIT | MSG_NOSIGNAL);
  if (sent < 0) {
    return -errno;
  }
  return sent == static_cast<ssize_t>(sizeof(frame)) ? 0 : -EIO;
}

int Socket::receive(can_frame & frame) const noexcept
{
  const ssize_t received = ::recv(descriptor_, &frame, sizeof(frame), MSG_DONTWAIT);
  if (received < 0) {
    return errno == EAGAIN || errno == EWOULDBLOCK ? 0 : -errno;
  }
  return received == static_cast<ssize_t>(sizeof(frame)) ? 1 : -EIO;
}

int Socket::wait_readable(int timeout_ms) const noexcept
{
  pollfd descriptor{descriptor_, POLLIN, 0};
  const int result = ::poll(&descriptor, 1U, timeout_ms);
  if (result < 0) {
    return -errno;
  }
  if (result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    return -EIO;
  }
  return result > 0 && (descriptor.revents & POLLIN) != 0 ? 1 : 0;
}

}  // namespace infra_socketcan
