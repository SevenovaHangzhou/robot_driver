#ifndef INFRA_MODBUS__CLIENT_HPP_
#define INFRA_MODBUS__CLIENT_HPP_

#include <sys/socket.h>

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace infra_modbus
{

// The device answered with a Modbus exception response; the connection remains usable.
class ModbusException : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Synchronous Modbus TCP client holding one long-lived connection to one endpoint.
// Not thread-safe and never for a real-time thread. No request is retried automatically.
class Client final
{
public:
  // `host` must be a numeric IPv4/IPv6 address. `timeout` bounds each transact() call,
  // including the connect when one is needed. Throws std::invalid_argument.
  Client(const std::string & host, std::uint16_t port, std::chrono::milliseconds timeout);
  ~Client() noexcept;
  Client(const Client &) = delete;
  Client & operator=(const Client &) = delete;

  // Sends `pdu` (function code first) to `unit` and returns the response PDU (function code
  // first). Connects on demand. Any transport, timeout or framing error closes the connection
  // and throws std::runtime_error, so a late reply can never be read as the next response;
  // the next call reconnects. An exception response throws ModbusException and keeps it open.
  std::vector<std::uint8_t> transact(std::uint8_t unit, const std::vector<std::uint8_t> & pdu);
  void close() noexcept;
  [[nodiscard]] bool connected() const noexcept;

private:
  using Clock = std::chrono::steady_clock;
  void connect(Clock::time_point deadline);
  void wait(short events, Clock::time_point deadline) const;
  void transfer(std::uint8_t * data, std::size_t size, bool sending, Clock::time_point deadline);

  sockaddr_storage address_{};
  socklen_t address_length_{0};
  std::chrono::milliseconds timeout_;
  int descriptor_{-1};
  std::uint16_t transaction_id_{0};
};

}  // namespace infra_modbus

#endif  // INFRA_MODBUS__CLIENT_HPP_
