#ifndef HW_CAN_DAMIAO__SOCKETCAN_HPP_
#define HW_CAN_DAMIAO__SOCKETCAN_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "infra_socketcan/socket.hpp"
#include "hw_can_damiao/protocol.hpp"

namespace hw_can_damiao
{

class SocketCan final
{
public:
  SocketCan() noexcept = default;

  SocketCan(const SocketCan &) = delete;
  SocketCan & operator=(const SocketCan &) = delete;

  void open(const std::string & interface_name, const std::vector<std::uint16_t> & receive_ids);
  void close() noexcept;
  [[nodiscard]] bool is_open() const noexcept;
  void send(const CanFrame & frame) const;
  [[nodiscard]] bool receive(CanFrame & frame) const;
  [[nodiscard]] bool wait_readable(int timeout_ms) const;

private:
  infra_socketcan::Socket socket_;
};

}  // namespace hw_can_damiao

#endif  // HW_CAN_DAMIAO__SOCKETCAN_HPP_
