#ifndef DM_SWERVE_DRIVER__SOCKETCAN_INTERFACE_HPP_
#define DM_SWERVE_DRIVER__SOCKETCAN_INTERFACE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "dm_swerve_driver/dm_frame_codec.hpp"

namespace dm_swerve_driver {

class SocketCanError : public std::runtime_error {
public:
  SocketCanError(std::string message, int error_number);

  [[nodiscard]] int error_number() const noexcept;

private:
  int error_number_{0};
};

struct SocketCanOptions {
  std::string interface_name;
  std::vector<std::uint16_t> receive_ids;
  std::chrono::microseconds write_timeout{2000};
  bool receive_own_messages{false};
};

struct ReceivedCanFrame {
  CanFrame frame;
  std::chrono::nanoseconds kernel_timestamp{0};
};

class SocketCanInterface final {
public:
  SocketCanInterface() noexcept = default;
  explicit SocketCanInterface(const SocketCanOptions & options);
  ~SocketCanInterface() noexcept;

  SocketCanInterface(const SocketCanInterface &) = delete;
  SocketCanInterface & operator=(const SocketCanInterface &) = delete;
  SocketCanInterface(SocketCanInterface && other) noexcept;
  SocketCanInterface & operator=(SocketCanInterface && other) noexcept;

  void open(const SocketCanOptions & options);
  void close() noexcept;

  [[nodiscard]] bool is_open() const noexcept;
  [[nodiscard]] int native_handle() const noexcept;

  void write_batch(const std::vector<CanFrame> & frames) const;
  [[nodiscard]] std::vector<ReceivedCanFrame> collect(
    std::size_t expected_count,
    std::chrono::steady_clock::time_point deadline) const;

private:
  int file_descriptor_{-1};
  std::chrono::microseconds write_timeout_{2000};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__SOCKETCAN_INTERFACE_HPP_
