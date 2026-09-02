#include "dm_swerve_driver/socketcan_interface.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace dm_swerve_driver {
namespace {

class UniqueFileDescriptor final {
public:
  explicit UniqueFileDescriptor(int descriptor) noexcept : descriptor_{descriptor} {}
  ~UniqueFileDescriptor() noexcept
  {
    if (descriptor_ >= 0) {
      static_cast<void>(::close(descriptor_));
    }
  }

  UniqueFileDescriptor(const UniqueFileDescriptor &) = delete;
  UniqueFileDescriptor & operator=(const UniqueFileDescriptor &) = delete;

  [[nodiscard]] int get() const noexcept {return descriptor_;}
  [[nodiscard]] int release() noexcept
  {
    const int descriptor{descriptor_};
    descriptor_ = -1;
    return descriptor;
  }

private:
  int descriptor_{-1};
};

struct NativeBatch {
  NativeBatch() = default;
  NativeBatch(const NativeBatch &) = delete;
  NativeBatch & operator=(const NativeBatch &) = delete;
  NativeBatch(NativeBatch &&) = delete;
  NativeBatch & operator=(NativeBatch &&) = delete;

  std::vector<can_frame> frames;
  std::vector<iovec> payloads;
  std::vector<mmsghdr> messages;
};

[[nodiscard]] std::string error_message(const std::string & action, int error_number)
{
  return action + ": " + std::strerror(error_number);
}

[[noreturn]] void throw_socket_error(const std::string & action, int error_number)
{
  throw SocketCanError{error_message(action, error_number), error_number};
}

void validate_options(const SocketCanOptions & options)
{
  if (options.interface_name.empty() || options.interface_name.size() >= IFNAMSIZ) {
    throw std::invalid_argument{"SocketCAN interface name is empty or too long"};
  }
  if (options.write_timeout < std::chrono::microseconds::zero()) {
    throw std::invalid_argument{"SocketCAN write timeout cannot be negative"};
  }
  if (std::any_of(options.receive_ids.begin(), options.receive_ids.end(), [](std::uint16_t id) {
      return id > CAN_SFF_MASK;
    }))
  {
    throw std::invalid_argument{"SocketCAN filter identifier exceeds 11 bits"};
  }
}

void validate_outgoing_frame(const CanFrame & frame)
{
  if (frame.id > CAN_SFF_MASK) {
    throw std::invalid_argument{"outgoing CAN identifier exceeds 11 bits"};
  }
  if (frame.length != CAN_MAX_DLEN) {
    throw std::invalid_argument{"DaMiao outgoing frames must contain eight bytes"};
  }
}

[[nodiscard]] int poll_until(
  int file_descriptor, short events, std::chrono::steady_clock::time_point deadline)
{
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return 0;
    }
    const auto remaining = deadline - now;
    const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
    const auto rounded_ms = remaining_ms +
      (remaining_ms < remaining ? std::chrono::milliseconds{1} : std::chrono::milliseconds{0});
    const int timeout_ms = static_cast<int>(std::min<std::int64_t>(
        rounded_ms.count(), std::numeric_limits<int>::max()));

    pollfd descriptor{file_descriptor, events, 0};
    const int result = ::poll(&descriptor, 1U, timeout_ms);
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result < 0) {
      throw_socket_error("poll SocketCAN socket", errno);
    }
    if (result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw_socket_error("SocketCAN socket reported an I/O error", EIO);
    }
    return result;
  }
}

[[nodiscard]] can_frame to_native_frame(const CanFrame & frame)
{
  validate_outgoing_frame(frame);
  can_frame native{};
  native.can_id = frame.id;
  native.can_dlc = frame.length;
  std::copy(frame.data.begin(), frame.data.end(), native.data);
  return native;
}

void configure_socket(int descriptor, const SocketCanOptions & options)
{
  const int receive_own_messages = options.receive_own_messages ? 1 : 0;
  if (::setsockopt(
      descriptor, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
      &receive_own_messages, sizeof(receive_own_messages)) < 0)
  {
    throw_socket_error("configure SocketCAN loopback reception", errno);
  }

  std::vector<can_filter> filters(options.receive_ids.size());
  std::transform(
    options.receive_ids.begin(), options.receive_ids.end(), filters.begin(),
    [](std::uint16_t id) {
      return can_filter{id, static_cast<canid_t>(CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG)};
    });
  const void * filter_data = filters.empty() ? nullptr : filters.data();
  const socklen_t filter_size = static_cast<socklen_t>(filters.size() * sizeof(can_filter));
  if (::setsockopt(descriptor, SOL_CAN_RAW, CAN_RAW_FILTER, filter_data, filter_size) < 0) {
    throw_socket_error("configure SocketCAN receive filters", errno);
  }

  const int enable_timestamps{1};
  if (::setsockopt(
      descriptor, SOL_SOCKET, SO_TIMESTAMPNS,
      &enable_timestamps, sizeof(enable_timestamps)) < 0)
  {
    throw_socket_error("enable SocketCAN kernel timestamps", errno);
  }
}

[[nodiscard]] int resolve_interface_index(int descriptor, const std::string & interface_name)
{
  ifreq interface_request{};
  std::strncpy(
    interface_request.ifr_name, interface_name.c_str(),
    sizeof(interface_request.ifr_name) - 1U);
  if (::ioctl(descriptor, SIOCGIFINDEX, &interface_request) < 0) {
    throw_socket_error("resolve SocketCAN interface " + interface_name, errno);
  }
  return interface_request.ifr_ifindex;
}

void bind_interface(int descriptor, int interface_index, const std::string & interface_name)
{
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_index;
  if (::bind(descriptor, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
    throw_socket_error("bind SocketCAN interface " + interface_name, errno);
  }
}

void prepare_native_batch(const std::vector<CanFrame> & frames, NativeBatch & batch)
{
  batch.frames.resize(frames.size());
  std::transform(frames.begin(), frames.end(), batch.frames.begin(), to_native_frame);
  batch.payloads.resize(frames.size());
  batch.messages.resize(frames.size());
  for (std::size_t index{0U}; index < frames.size(); ++index) {
    batch.payloads[index] = iovec{&batch.frames[index], sizeof(can_frame)};
    batch.messages[index].msg_hdr.msg_iov = &batch.payloads[index];
    batch.messages[index].msg_hdr.msg_iovlen = 1U;
  }
}

void send_native_batch(
  int descriptor, NativeBatch & batch,
  std::chrono::steady_clock::time_point deadline)
{
  std::size_t sent_count{0U};
  while (sent_count < batch.messages.size()) {
    const auto remaining = static_cast<unsigned int>(batch.messages.size() - sent_count);
    const int result = ::sendmmsg(
      descriptor, batch.messages.data() + sent_count, remaining, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (result > 0) {
      const auto newly_sent = static_cast<std::size_t>(result);
      for (std::size_t index{sent_count}; index < sent_count + newly_sent; ++index) {
        if (batch.messages[index].msg_len != sizeof(can_frame)) {
          throw_socket_error("partial SocketCAN frame write", EMSGSIZE);
        }
      }
      sent_count += newly_sent;
    } else if (result == 0) {
      throw_socket_error("SocketCAN batch sent zero frames", EIO);
    } else if (errno != EINTR) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOBUFS) {
        throw_socket_error("send SocketCAN frame batch", errno);
      }
      if (poll_until(descriptor, POLLOUT, deadline) == 0) {
        throw_socket_error("SocketCAN frame batch timed out", ETIMEDOUT);
      }
    }
  }
}

[[nodiscard]] ReceivedCanFrame receive_one(int file_descriptor)
{
  can_frame native{};
  iovec payload{&native, sizeof(native)};
  alignas(cmsghdr) std::array<char, CMSG_SPACE(sizeof(timespec))> control{};
  msghdr message{};
  message.msg_iov = &payload;
  message.msg_iovlen = 1U;
  message.msg_control = control.data();
  message.msg_controllen = control.size();

  const ssize_t received = ::recvmsg(file_descriptor, &message, MSG_DONTWAIT);
  if (received < 0) {
    throw_socket_error("receive SocketCAN frame", errno);
  }
  if (received != static_cast<ssize_t>(sizeof(native))) {
    throw_socket_error("receive partial SocketCAN frame", EMSGSIZE);
  }
  if ((native.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U) {
    throw_socket_error("receive non-standard SocketCAN data frame", EPROTO);
  }

  std::chrono::nanoseconds timestamp{0};
  for (cmsghdr * header = CMSG_FIRSTHDR(&message); header != nullptr;
    header = CMSG_NXTHDR(&message, header))
  {
    if (header->cmsg_level == SOL_SOCKET && header->cmsg_type == SO_TIMESTAMPNS) {
      timespec kernel_time{};
      std::memcpy(&kernel_time, CMSG_DATA(header), sizeof(kernel_time));
      timestamp = std::chrono::seconds{kernel_time.tv_sec} +
        std::chrono::nanoseconds{kernel_time.tv_nsec};
      break;
    }
  }

  CanFrame frame{};
  frame.id = static_cast<std::uint16_t>(native.can_id & CAN_SFF_MASK);
  frame.length = native.can_dlc;
  std::copy(native.data, native.data + CAN_MAX_DLEN, frame.data.begin());
  return ReceivedCanFrame{frame, timestamp};
}

}  // namespace

SocketCanError::SocketCanError(std::string message, int error_number)
: std::runtime_error{std::move(message)}, error_number_{error_number}
{
}

int SocketCanError::error_number() const noexcept
{
  return error_number_;
}

SocketCanInterface::SocketCanInterface(const SocketCanOptions & options)
{
  open(options);
}

SocketCanInterface::~SocketCanInterface() noexcept
{
  close();
}

SocketCanInterface::SocketCanInterface(SocketCanInterface && other) noexcept
: file_descriptor_{other.file_descriptor_}, write_timeout_{other.write_timeout_}
{
  other.file_descriptor_ = -1;
}

SocketCanInterface & SocketCanInterface::operator=(SocketCanInterface && other) noexcept
{
  if (this != &other) {
    close();
    file_descriptor_ = other.file_descriptor_;
    write_timeout_ = other.write_timeout_;
    other.file_descriptor_ = -1;
  }
  return *this;
}

void SocketCanInterface::open(const SocketCanOptions & options)
{
  validate_options(options);
  const int descriptor = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (descriptor < 0) {
    throw_socket_error("create SocketCAN socket", errno);
  }
  UniqueFileDescriptor pending{descriptor};
  configure_socket(pending.get(), options);
  const int interface_index{resolve_interface_index(pending.get(), options.interface_name)};
  bind_interface(pending.get(), interface_index, options.interface_name);

  close();
  file_descriptor_ = pending.release();
  write_timeout_ = options.write_timeout;
}

void SocketCanInterface::close() noexcept
{
  if (file_descriptor_ >= 0) {
    static_cast<void>(::close(file_descriptor_));
    file_descriptor_ = -1;
  }
}

bool SocketCanInterface::is_open() const noexcept
{
  return file_descriptor_ >= 0;
}

int SocketCanInterface::native_handle() const noexcept
{
  return file_descriptor_;
}

void SocketCanInterface::write_batch(const std::vector<CanFrame> & frames) const
{
  if (frames.empty()) {
    return;
  }
  if (!is_open()) {
    throw_socket_error("write on closed SocketCAN socket", EBADF);
  }
  if (frames.size() > std::numeric_limits<unsigned int>::max()) {
    throw std::length_error{"SocketCAN batch exceeds sendmmsg capacity"};
  }

  NativeBatch batch;
  prepare_native_batch(frames, batch);
  const auto deadline = std::chrono::steady_clock::now() + write_timeout_;
  send_native_batch(file_descriptor_, batch, deadline);
}

std::vector<ReceivedCanFrame> SocketCanInterface::collect(
  std::size_t expected_count, std::chrono::steady_clock::time_point deadline) const
{
  if (expected_count == 0U) {
    return {};
  }
  if (!is_open()) {
    throw_socket_error("read on closed SocketCAN socket", EBADF);
  }

  std::vector<ReceivedCanFrame> received;
  received.reserve(expected_count);
  while (received.size() < expected_count) {
    if (poll_until(file_descriptor_, POLLIN, deadline) == 0) {
      break;
    }
    try {
      received.push_back(receive_one(file_descriptor_));
    } catch (const SocketCanError & error) {
      if (error.error_number() == EINTR || error.error_number() == EAGAIN ||
        error.error_number() == EWOULDBLOCK)
      {
        continue;
      }
      throw;
    }
  }
  return received;
}

}  // namespace dm_swerve_driver
