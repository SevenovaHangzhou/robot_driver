#include "robot_hw_can/socketcan.hpp"

#include <linux/can.h>

#include <algorithm>
#include <cerrno>
#include <stdexcept>
#include <system_error>

namespace robot_hw_can
{
namespace
{

[[noreturn]] void throw_system_error(int error, const char * action)
{
  throw std::system_error{error, std::generic_category(), action};
}

}  // namespace

void SocketCan::open(
  const std::string & interface_name, const std::vector<std::uint16_t> & receive_ids)
{
  if (receive_ids.empty() ||
    std::any_of(receive_ids.begin(), receive_ids.end(), [](std::uint16_t id) {
      return id > CAN_SFF_MASK;
    }))
  {
    throw std::invalid_argument{"SocketCAN receive IDs must be non-empty standard IDs"};
  }
  std::vector<can_filter> filters;
  filters.reserve(receive_ids.size());
  for (const auto id : receive_ids) {
    filters.push_back(
      can_filter{id, static_cast<canid_t>(CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG)});
  }
  socket_.open(interface_name, filters);
}

void SocketCan::close() noexcept
{
  socket_.close();
}

bool SocketCan::is_open() const noexcept
{
  return socket_.is_open();
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
  const int result = socket_.send(native);
  if (result == -EIO) {
    throw std::runtime_error{"partial SocketCAN frame write"};
  }
  if (result < 0) {
    throw_system_error(-result, "send SocketCAN frame");
  }
}

bool SocketCan::receive(CanFrame & frame) const
{
  if (!is_open()) {
    throw std::logic_error{"SocketCAN socket is not open"};
  }
  can_frame native{};
  const int result = socket_.receive(native);
  if (result == 0) {
    return false;
  }
  if (result < 0 && result != -EIO) {
    throw_system_error(-result, "receive SocketCAN frame");
  }
  if (
    result < 0 || (native.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U ||
    native.can_dlc != CAN_MAX_DLEN) {
    throw std::runtime_error{"received malformed DaMiao CAN frame"};
  }
  frame.id = static_cast<std::uint16_t>(native.can_id & CAN_SFF_MASK);
  frame.length = native.can_dlc;
  std::copy(native.data, native.data + CAN_MAX_DLEN, frame.data.begin());
  return true;
}

bool SocketCan::wait_readable(int timeout_ms) const
{
  int result{0};
  do {
    result = socket_.wait_readable(timeout_ms);
  } while (result == -EINTR);
  if (result == -EIO) {
    throw std::runtime_error{"SocketCAN socket reported an I/O error"};
  }
  if (result < 0) {
    throw_system_error(-result, "poll SocketCAN socket");
  }
  return result > 0;
}

}  // namespace robot_hw_can
