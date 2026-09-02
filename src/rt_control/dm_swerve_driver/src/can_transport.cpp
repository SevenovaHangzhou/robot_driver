#include "dm_swerve_driver/can_transport.hpp"

#include <utility>

namespace dm_swerve_driver {

SocketCanTransport::SocketCanTransport(SocketCanOptions options)
: options_{std::move(options)}
{
}

void SocketCanTransport::open()
{
  interface_.open(options_);
}

void SocketCanTransport::close() noexcept
{
  interface_.close();
}

bool SocketCanTransport::is_open() const noexcept
{
  return interface_.is_open();
}

void SocketCanTransport::write_batch(const std::vector<CanFrame> & frames)
{
  interface_.write_batch(frames);
}

std::vector<ReceivedCanFrame> SocketCanTransport::collect(
  std::size_t expected_count,
  std::chrono::steady_clock::time_point deadline)
{
  return interface_.collect(expected_count, deadline);
}

}  // namespace dm_swerve_driver
