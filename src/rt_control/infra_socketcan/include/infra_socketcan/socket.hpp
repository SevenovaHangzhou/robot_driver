#ifndef INFRA_SOCKETCAN__SOCKET_HPP_
#define INFRA_SOCKETCAN__SOCKET_HPP_

#include <linux/can.h>

#include <string>
#include <vector>

namespace infra_socketcan
{

// Non-blocking classic-CAN CAN_RAW socket bound to one interface.
// The socket never receives its own transmitted frames; other local sockets (candump) still
// see them. Reconnect policy, filter construction and frame decoding stay with the caller.
class Socket final
{
public:
  Socket() noexcept = default;
  ~Socket() noexcept;
  Socket(const Socket &) = delete;
  Socket & operator=(const Socket &) = delete;
  Socket(Socket && other) noexcept;
  Socket & operator=(Socket && other) noexcept;

  // Throws std::invalid_argument or std::system_error; an already open socket is kept on failure.
  void open(const std::string & interface_name, const std::vector<can_filter> & filters);
  void close() noexcept;
  [[nodiscard]] bool is_open() const noexcept;
  // Kernel index of the bound interface; it changes when a USB adapter is re-enumerated.
  [[nodiscard]] int interface_index() const noexcept;

  // The calls below never block or throw, so they may run in a real-time loop.
  // send: 0 on success, -errno on failure.
  [[nodiscard]] int send(const can_frame & frame) const noexcept;
  // receive: 1 with `frame` filled, 0 when no frame is queued, -errno on failure.
  [[nodiscard]] int receive(can_frame & frame) const noexcept;
  // wait_readable: 1 readable, 0 timed out, -errno on failure (-EINTR is returned, not retried;
  // -EIO when poll reports POLLERR, POLLHUP or POLLNVAL).
  [[nodiscard]] int wait_readable(int timeout_ms) const noexcept;

private:
  int descriptor_{-1};
  int interface_index_{0};
};

}  // namespace infra_socketcan

#endif  // INFRA_SOCKETCAN__SOCKET_HPP_
