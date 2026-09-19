#ifndef ROBOT_HW_CAN__SOCKETCAN_HPP_
#define ROBOT_HW_CAN__SOCKETCAN_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "robot_hw_can/protocol.hpp"

namespace robot_hw_can
{

class SocketCan final
{
public:
  SocketCan() noexcept = default;
  ~SocketCan() noexcept;

  SocketCan(const SocketCan &) = delete;
  SocketCan & operator=(const SocketCan &) = delete;

  void open(const std::string & interface_name, const std::vector<std::uint16_t> & receive_ids);
  void close() noexcept;
  [[nodiscard]] bool is_open() const noexcept;
  void send(const CanFrame & frame) const;
  [[nodiscard]] bool receive(CanFrame & frame) const;
  [[nodiscard]] bool wait_readable(int timeout_ms) const;

private:
  int descriptor_{-1};
};

}  // namespace robot_hw_can

#endif  // ROBOT_HW_CAN__SOCKETCAN_HPP_
