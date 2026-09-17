#include "modbus_transport.hpp"
#include <iostream>
#include <limits>
#include <thread>

using namespace modbus_tcp_rtu485_led;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Function>
void rejects(Function function)
{
  bool failed = false;
  try {function();} catch (const std::exception &) {failed = true;}
  require(failed, "Expected rejection");
}

void response_case(std::vector<uint8_t> response, bool valid, bool fragmented = false)
{
  std::cout << "response bytes=" << response.size() << " valid=" << valid
    << " fragmented=" << fragmented << std::endl;
  int pair[2];
  require(::socketpair(AF_UNIX, SOCK_STREAM, 0, pair) == 0, "socketpair failed");
  Socket client(pair[0]), server(pair[1]);
  std::exception_ptr peer_error;
  std::thread peer([&]() {
      try {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        std::array<uint8_t, 21> request{};
        transfer(server.get(), request.data(), request.size(), false, deadline);
        for (size_t i = 0; i < response.size();) {
          const auto size = fragmented ? size_t{1} : response.size();
          transfer(server.get(), response.data() + i, size, true, deadline);
          i += size;
          if (fragmented) {std::this_thread::sleep_for(std::chrono::milliseconds(2));}
        }
        ::shutdown(server.get(), SHUT_WR);
      } catch (...) {peer_error = std::current_exception();}
    });
  bool success = true;
  try {
    exchange(client.get(), color_request(0x1234, 1, {255, 0, 128, 0}),
      std::chrono::steady_clock::now() + std::chrono::seconds(1));
  } catch (const std::exception & error) {
    std::cout << "client: " << error.what() << std::endl;
    success = false;
  }
  peer.join();
  if (peer_error) {std::rethrow_exception(peer_error);}
  require(success == valid, "Unexpected response result");
}

void protocol_tests()
{
  const std::vector<uint8_t> ok{0x12, 0x34, 0, 0, 0, 6, 1, 0x10, 0, 0, 0, 4};
  response_case(ok, true);
  response_case(ok, true, true);
  for (const size_t offset : {size_t{0}, size_t{2}, size_t{4}, size_t{5},
    size_t{6}, size_t{7}, size_t{9}, size_t{11}})
  {
    auto invalid = ok;
    invalid[offset] ^= 1;
    response_case(invalid, false);
  }
  response_case({0x12, 0x34, 0, 0, 0, 3, 1, 0x90, 2}, false);
  response_case({0x12, 0x34, 0, 0, 0, 6, 1, 0x10}, false);
  response_case({}, false);
  response_case(ok, true);  // A failed request must not poison the next transaction.
}

void parameter_tests()
{
  const std::vector<int64_t> ports{502, 503, 504, 505}, units{1, 1, 1, 1};
  validate_config("127.0.0.1", ports, units, 500);
  rejects([&]() {validate_config("bad-ip", ports, units, 500);});
  rejects([&]() {validate_config("127.0.0.1", {}, units, 500);});
  rejects([&]() {validate_config("127.0.0.1", ports, {}, 500);});
  for (const auto value : {int64_t{-1}, int64_t{0}, int64_t{65536}}) {
    auto invalid = ports; invalid[0] = value;
    rejects([&]() {validate_config("127.0.0.1", invalid, units, 500);});
  }
  for (const auto value : {int64_t{-1}, int64_t{0}, int64_t{248}, int64_t{256}}) {
    auto invalid = units; invalid[0] = value;
    rejects([&]() {validate_config("127.0.0.1", ports, invalid, 500);});
  }
  for (const auto value : {int64_t{-1}, int64_t{0}, int64_t{60001}}) {
    rejects([&]() {validate_config("127.0.0.1", ports, units, value);});
  }
  validate_config("127.0.0.1", {1, 65535, 502, 503}, {1, 247, 1, 1}, 1);
  require(brightness(-1) == 0 && brightness(2) == 255 && brightness(0.5F) == 128,
    "Brightness conversion failed");
  require(brightness(std::numeric_limits<float>::quiet_NaN()) == 0 &&
    brightness(std::numeric_limits<float>::infinity()) == 0, "Non-finite conversion failed");
  require(color_request(0x1234, 1, {255, 0, 128, 0}) == std::vector<uint8_t>({
      0x12, 0x34, 0, 0, 0, 15, 1, 0x10, 0, 0, 0, 4, 8, 0, 255, 0, 0, 0, 128, 0, 0}),
    "Request encoding failed");
}

void timeout_tests()
{
  int pair[2];
  require(::socketpair(AF_UNIX, SOCK_STREAM, 0, pair) == 0, "socketpair failed");
  Socket client(pair[0]), server(pair[1]);
  const auto start = std::chrono::steady_clock::now();
  rejects([&]() {exchange(client.get(), color_request(1, 1, {0, 0, 0, 0}),
      start + std::chrono::milliseconds(40));});
  require(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500),
    "Timeout was not bounded");
  // A full send buffer exercises partial writes and send-side deadline handling.
  std::vector<uint8_t> large(4 * 1024 * 1024, 0);
  rejects([&]() {transfer(client.get(), large.data(), large.size(), true,
      std::chrono::steady_clock::now() + std::chrono::milliseconds(40));});
}

int main()
{
  try {
    parameter_tests();
    protocol_tests();
    timeout_tests();
    std::cout << "PASS: parameters, encoding, fragmented responses, exceptions, disconnect, "
      "recovery, receive/send deadlines\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
