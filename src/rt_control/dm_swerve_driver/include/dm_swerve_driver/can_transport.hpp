#ifndef DM_SWERVE_DRIVER__CAN_TRANSPORT_HPP_
#define DM_SWERVE_DRIVER__CAN_TRANSPORT_HPP_

#include <chrono>
#include <cstddef>
#include <vector>

#include "dm_swerve_driver/socketcan_interface.hpp"

namespace dm_swerve_driver {

class CanTransport {
public:
  virtual ~CanTransport() = default;

  virtual void open() = 0;
  virtual void close() noexcept = 0;
  [[nodiscard]] virtual bool is_open() const noexcept = 0;
  virtual void write_batch(const std::vector<CanFrame> & frames) = 0;
  [[nodiscard]] virtual std::vector<ReceivedCanFrame> collect(
    std::size_t expected_count,
    std::chrono::steady_clock::time_point deadline) = 0;
};

class SocketCanTransport final : public CanTransport {
public:
  explicit SocketCanTransport(SocketCanOptions options);

  void open() override;
  void close() noexcept override;
  [[nodiscard]] bool is_open() const noexcept override;
  void write_batch(const std::vector<CanFrame> & frames) override;
  [[nodiscard]] std::vector<ReceivedCanFrame> collect(
    std::size_t expected_count,
    std::chrono::steady_clock::time_point deadline) override;

private:
  SocketCanOptions options_;
  SocketCanInterface interface_;
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__CAN_TRANSPORT_HPP_
