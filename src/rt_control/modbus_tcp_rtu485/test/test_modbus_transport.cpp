#include "modbus_transport.hpp"
#include "ultrasonic_protocol.hpp"
#include "ultrasonic_range.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <thread>

using namespace modbus_tcp_rtu485;

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

std::vector<uint8_t> exchange_case(
  const std::vector<uint8_t> & request, std::vector<uint8_t> response,
  bool fragmented = false)
{
  int pair[2];
  require(::socketpair(AF_UNIX, SOCK_STREAM, 0, pair) == 0, "socketpair failed");
  Socket client(pair[0]), server(pair[1]);
  std::exception_ptr peer_error;
  std::thread peer([&]() {
      try {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        std::vector<uint8_t> received(request.size());
        transfer(server.get(), received.data(), received.size(), false, deadline);
        require(received == request, "Peer received an unexpected request");
        for (size_t i = 0; i < response.size();) {
          const size_t count = fragmented ? size_t{1} : response.size();
          transfer(server.get(), response.data() + i, count, true, deadline);
          i += count;
          if (fragmented) {std::this_thread::sleep_for(std::chrono::milliseconds(1));}
        }
        ::shutdown(server.get(), SHUT_WR);
      } catch (...) {peer_error = std::current_exception();}
    });
  std::vector<uint8_t> result;
  try {
    result = exchange(
      client.get(), request, std::chrono::steady_clock::now() + std::chrono::seconds(1));
  } catch (...) {
    peer.join();
    if (peer_error) {std::rethrow_exception(peer_error);}
    throw;
  }
  peer.join();
  if (peer_error) {std::rethrow_exception(peer_error);}
  return result;
}

void parameter_and_encoding_tests()
{
  const std::vector<int64_t> ports{502, 502, 502, 502, 502, 502};
  const std::vector<int64_t> units{1, 2, 3, 4, 5, 6};
  validate_led_config("127.0.0.1", ports, units, 500);
  validate_endpoint("127.0.0.1", 504, 1, 500);
  validate_e08_unit_ids({1, 6});
  rejects([&]() {validate_led_config("bad-ip", ports, units, 500);});
  rejects([&]() {validate_led_config("127.0.0.1", {}, units, 500);});
  rejects([&]() {validate_led_config("127.0.0.1", ports, {}, 500);});
  for (const auto value : {int64_t{-1}, int64_t{0}, int64_t{65536}}) {
    rejects([&]() {validate_endpoint("127.0.0.1", value, 1, 500);});
  }
  for (const auto value : {int64_t{-1}, int64_t{0}, int64_t{248}, int64_t{256}}) {
    rejects([&]() {validate_endpoint("127.0.0.1", 502, value, 500);});
  }
  for (const auto value : {int64_t{-1}, int64_t{0}, int64_t{60001}}) {
    rejects([&]() {validate_endpoint("127.0.0.1", 502, 1, value);});
  }
  rejects([]() {validate_e08_unit_ids({1});});
  rejects([]() {validate_e08_unit_ids({1, 1});});
  rejects([]() {validate_e08_unit_ids({1, 2});});
  require(brightness(-1) == 0 && brightness(2) == 255 && brightness(0.5F) == 128,
    "Brightness conversion failed");
  require(brightness(std::numeric_limits<float>::quiet_NaN()) == 0 &&
    brightness(std::numeric_limits<float>::infinity()) == 0,
    "Non-finite conversion failed");
  require(color_request(0x1234, 1, {255, 0, 128, 0}) == std::vector<uint8_t>({
      0x12, 0x34, 0, 0, 0, 15, 1, 0x10, 0, 0, 0, 4, 8, 0, 255, 0, 0, 0, 128, 0, 0}),
    "Color request encoding failed");
  require(read_holding_request(1, 1, 0x0106, 4) == std::vector<uint8_t>({
      0, 1, 0, 0, 0, 6, 1, 3, 1, 6, 0, 4}),
    "First E08 request encoding failed");
  require(read_holding_request(2, 6, 0x0106, 4) == std::vector<uint8_t>({
      0, 2, 0, 0, 0, 6, 6, 3, 1, 6, 0, 4}),
    "Second E08 request encoding failed");
  rejects([]() {read_holding_request(1, 1, 0, 0);});
  rejects([]() {read_holding_request(1, 1, 0, 126);});
}

void protocol_tests()
{
  const auto write_request = color_request(0x1234, 1, {255, 0, 128, 0});
  const std::vector<uint8_t> write_response{0x12, 0x34, 0, 0, 0, 6, 1, 0x10, 0, 0, 0, 4};
  std::cout << "  write\n";
  require(exchange_case(write_request, write_response) == write_response,
    "Write response failed");
  std::cout << "  write fragmented\n";
  require(exchange_case(write_request, write_response, true) == write_response,
    "Fragmented response failed");

  const auto read_request = read_holding_request(1, 1, 0x0106, 4);
  const std::vector<uint8_t> read_response{
    0, 1, 0, 0, 0, 11, 1, 3, 8, 0, 0x19, 0, 0x47, 0, 0x13, 0xFF, 0xFD};
  std::cout << "  read fragmented\n";
  const auto received = exchange_case(read_request, read_response, true);
  require(parse_holding_response(received, 4) ==
    std::vector<uint16_t>({0x0019, 0x0047, 0x0013, 0xFFFD}),
    "Holding register parsing failed");

  std::cout << "  invalid headers\n";
  for (const size_t offset : {size_t{0}, size_t{2}, size_t{4}, size_t{6}, size_t{7}}) {
    auto invalid = read_response;
    invalid[offset] ^= 1;
    rejects([&]() {exchange_case(read_request, invalid);});
  }
  std::cout << "  exception and disconnect\n";
  rejects([&]() {exchange_case(read_request, {0, 1, 0, 0, 0, 3, 1, 0x83, 2});});
  rejects([&]() {parse_holding_response(read_response, 3);});
  rejects([&]() {exchange_case(read_request, {});});
  std::cout << "  recovery\n";
  require(exchange_case(read_request, read_response) == read_response,
    "A failed request poisoned the next transaction");
}

void ultrasonic_decode_tests()
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto valid = decode_ultrasonic(25, 0.01F, 3.5F);
  require(valid.status == "ok" && valid.diagnostic_level == DiagnosticStatus::OK &&
    std::fabs(valid.range_m - 0.025F) < 1.0e-6F, "Valid distance decode failed");
  const auto no_target = decode_ultrasonic(0xFFFD, 0.01F, 3.5F);
  require(std::isinf(no_target.range_m) && no_target.diagnostic_level == DiagnosticStatus::OK &&
    no_target.status == "no_target", "No-target decode failed");
  require(decode_ultrasonic(0xFFFE, 0.01F, 3.5F).status == "interference",
    "Interference decode failed");
  require(decode_ultrasonic(0xFFFF, 0.01F, 3.5F).diagnostic_level == DiagnosticStatus::ERROR,
    "Timeout decode failed");
  require(decode_ultrasonic(0xEEEE, 0.01F, 3.5F).status == "checksum_error",
    "Checksum decode failed");
  require(decode_ultrasonic(0, 0.01F, 3.5F).status == "out_of_range",
    "Out-of-range decode failed");
}

void ultrasonic_range_message_tests()
{
  constexpr float field_of_view_rad = 0.6981317008F;
  const std::vector<std::string> frame_ids{
    "ultrasonic_channel_1_link", "ultrasonic_channel_2_link",
    "ultrasonic_channel_3_link", "ultrasonic_channel_4_link",
    "ultrasonic_channel_5_link", "ultrasonic_channel_6_link",
    "ultrasonic_channel_7_link", "ultrasonic_channel_8_link"};

  validate_ultrasonic_config(frame_ids, field_of_view_rad, 0.01F, 3.5F);
  rejects([&]() {validate_ultrasonic_config({}, field_of_view_rad, 0.01F, 3.5F);});
  rejects([&]() {
      validate_ultrasonic_config(
        {"ultrasonic_channel_1_link", "", "ultrasonic_channel_3_link",
          "ultrasonic_channel_4_link", "ultrasonic_channel_5_link",
          "ultrasonic_channel_6_link", "ultrasonic_channel_7_link",
          "ultrasonic_channel_8_link"}, field_of_view_rad, 0.01F, 3.5F);
    });
  rejects([&]() {validate_ultrasonic_config(frame_ids, 0.0F, 0.01F, 3.5F);});
  rejects([&]() {validate_ultrasonic_config(frame_ids, field_of_view_rad, 0.01F, 1.5F);});

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 123456789U;
  const auto message = make_range_message(
    stamp, frame_ids[0], decode_ultrasonic(500, 0.01F, 3.5F),
    field_of_view_rad, 0.01F, 3.5F);
  require(message.header.stamp == stamp, "Range timestamp was not preserved");
  require(message.header.frame_id == frame_ids[0], "Range frame_id was not preserved");
  require(message.radiation_type == sensor_msgs::msg::Range::ULTRASOUND,
    "Range radiation type must be ultrasound");
  require(std::fabs(message.field_of_view - field_of_view_rad) < 1.0e-6F,
    "Range field of view must be 60 degrees");
  require(std::fabs(message.min_range - 0.01F) < 1.0e-6F, "Range minimum is incorrect");
  require(std::fabs(message.max_range - 3.5F) < 1.0e-6F, "Range maximum is incorrect");
  require(std::fabs(message.range - 0.5F) < 1.0e-6F, "Range distance is incorrect");

  const auto no_target_message = make_range_message(
    stamp, frame_ids[1], decode_ultrasonic(0xFFFD, 0.01F, 3.5F),
    field_of_view_rad, 0.01F, 3.5F);
  require(std::isinf(no_target_message.range) && no_target_message.range > 0.0F,
    "No-target Range value must be positive infinity");
}

void timeout_tests()
{
  int pair[2];
  require(::socketpair(AF_UNIX, SOCK_STREAM, 0, pair) == 0, "socketpair failed");
  Socket client(pair[0]), server(pair[1]);
  const auto start = std::chrono::steady_clock::now();
  rejects([&]() {exchange(client.get(), read_holding_request(1, 1, 0x0106, 4),
      start + std::chrono::milliseconds(40));});
  require(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500),
    "Timeout was not bounded");
  std::vector<uint8_t> large(4 * 1024 * 1024, 0);
  rejects([&]() {transfer(client.get(), large.data(), large.size(), true,
      std::chrono::steady_clock::now() + std::chrono::milliseconds(40));});
}

int main()
{
  try {
    std::cout << "parameters and encoding\n";
    parameter_and_encoding_tests();
    std::cout << "protocol\n";
    protocol_tests();
    std::cout << "ultrasonic decode\n";
    ultrasonic_decode_tests();
    std::cout << "ultrasonic Range message\n";
    ultrasonic_range_message_tests();
    std::cout << "timeouts\n";
    timeout_tests();
    std::cout << "PASS: Modbus read/write transport and ultrasonic decoding\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
