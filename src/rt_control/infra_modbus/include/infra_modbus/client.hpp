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
  // including the connect when one is needed. Empty interface uses host routing; a specified
  // interface is bound on every new socket and never falls back if binding fails.
  // Changing this constructor requires rebuilding callers linked against this library.
  // Throws std::invalid_argument for malformed configuration.
  Client(
    const std::string & host, std::uint16_t port, std::chrono::milliseconds timeout,
    const std::string & interface = "");
  ~Client() noexcept;
  Client(const Client &) = delete;
  Client & operator=(const Client &) = delete;

  // Sends `pdu` (function code first) to `unit` and returns the response PDU (function code
  // first). Connects on demand. Any transport, timeout or framing error throws
  // std::runtime_error and closes the connection, so the next call reconnects, with one
  // exception: a timeout after the request was sent and before any reply byte arrived keeps
  // the connection if the previous call did not also end that way (a second consecutive such
  // timeout closes it). An exception response throws ModbusException and keeps it open.
  // A late reply to an earlier, abandoned request (transaction id among the last
  // kStaleTransactionWindow ids sent) is discarded and the call keeps waiting for its own reply:
  // some TCP-to-RTU gateways queue such replies and deliver them on the next connection. Any
  // other transaction id is a framing error. A late reply is never returned as a response.
  std::vector<std::uint8_t> transact(std::uint8_t unit, const std::vector<std::uint8_t> & pdu);
  // Same, with a different timeout for this call only (e.g. a short best-effort write on exit).
  std::vector<std::uint8_t> transact(
    std::uint8_t unit, const std::vector<std::uint8_t> & pdu, std::chrono::milliseconds timeout);
  void close() noexcept;
  [[nodiscard]] bool connected() const noexcept;
  // Minimum idle time between the end of one transact() and the start of the next request on
  // this client (default 0). The wait precedes and is not counted in the call timeout. Needed
  // by RS-485 gateways that drop a request sent immediately after the previous reply.
  // Throws std::invalid_argument when negative.
  void set_min_request_gap(std::chrono::milliseconds gap);

  static constexpr std::uint16_t kStaleTransactionWindow{64U};

private:
  using Clock = std::chrono::steady_clock;
  void connect(Clock::time_point deadline);
  void wait(short events, Clock::time_point deadline) const;
  void transfer(std::uint8_t * data, std::size_t size, bool sending, Clock::time_point deadline);
  std::vector<std::uint8_t> exchange(
    std::uint8_t unit, const std::vector<std::uint8_t> & pdu, Clock::time_point deadline);

  std::string interface_;
  sockaddr_storage address_{};
  socklen_t address_length_{0};
  std::chrono::milliseconds timeout_;
  int descriptor_{-1};
  std::uint16_t transaction_id_{0};
  std::uint8_t consecutive_reply_timeouts_{0};
  std::chrono::milliseconds min_request_gap_{0};
  Clock::time_point last_transaction_end_{};
};

}  // namespace infra_modbus

#endif  // INFRA_MODBUS__CLIENT_HPP_
