#include "periph_led_ultrasonic/protocol/modbus_transport.hpp"
#include "periph_led_ultrasonic/protocol/ultrasonic_protocol.hpp"
#include "periph_led_ultrasonic/ros_conversion.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cmath>
#include <iostream>
#include <limits>

using namespace periph_led_ultrasonic;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template <typename Function>
void rejects(Function function)
{
  bool failed = false;
  try {
    function();
  } catch (const std::exception &) {
    failed = true;
  }
  require(failed, "Expected rejection");
}

void parameter_and_encoding_tests()
{
  const std::vector<int64_t> ports{502, 502, 502, 502, 502, 502};
  const std::vector<int64_t> units{1, 2, 3, 4, 5, 6};
  validate_led_config(ports, units, 500);
  validate_endpoint(504, 1, 500);
  validate_e08_unit_ids({1, 6});
  rejects([&]() { validate_led_config({}, units, 500); });
  rejects([&]() { validate_led_config(ports, {}, 500); });
  for (const auto value : {int64_t{-1}, int64_t{0}, int64_t{65536}}) {
    rejects([&]() { validate_endpoint(value, 1, 500); });
  }
  for (const auto value : {int64_t{-1}, int64_t{0}, int64_t{248}, int64_t{256}}) {
    rejects([&]() { validate_endpoint(502, value, 500); });
  }
  for (const auto value : {int64_t{-1}, int64_t{0}, int64_t{60001}}) {
    rejects([&]() { validate_endpoint(502, 1, value); });
  }
  rejects([]() { validate_e08_unit_ids({1}); });
  rejects([]() { validate_e08_unit_ids({1, 1}); });
  rejects([]() { validate_e08_unit_ids({1, 2}); });
  require(
    brightness(-1) == 0 && brightness(2) == 255 && brightness(0.5F) == 128,
    "Brightness conversion failed");
  require(
    brightness(std::numeric_limits<float>::quiet_NaN()) == 0 &&
      brightness(std::numeric_limits<float>::infinity()) == 0,
    "Non-finite conversion failed");
  require(
    led_exit_color(false) == std::array<uint8_t, 4>({0, 0, 0, 0}), "Normal exit color must be off");
  require(
    led_exit_color(true) == std::array<uint8_t, 4>({255, 0, 0, 0}),
    "Abnormal exit color must be red");
  require(
    color_pdu({255, 0, 128, 0}) ==
      std::vector<uint8_t>({0x10, 0, 0, 0, 4, 8, 0, 255, 0, 0, 0, 128, 0, 0}),
    "Color request encoding failed");
  require(
    read_holding_pdu(0x0106, 4) == std::vector<uint8_t>({3, 1, 6, 0, 4}),
    "E08 request encoding failed");
  rejects([]() { read_holding_pdu(0, 0); });
  rejects([]() { read_holding_pdu(0, 126); });
}

void reply_tests()
{
  check_color_ack({0x10, 0, 0, 0, 4});
  rejects([]() { check_color_ack({0x10, 0, 0, 0, 3}); });
  rejects([]() { check_color_ack({0x10, 0, 1, 0, 4}); });
  rejects([]() { check_color_ack({0x06, 0, 0, 0, 4}); });

  const std::vector<uint8_t> reply{3, 8, 0, 0x19, 0, 0x47, 0, 0x13, 0xFF, 0xFD};
  require(
    parse_holding_reply(reply, 4) == std::vector<uint16_t>({0x0019, 0x0047, 0x0013, 0xFFFD}),
    "Holding register parsing failed");
  rejects([&]() { parse_holding_reply(reply, 3); });
  auto wrong_count = reply;
  wrong_count[1] = 6;
  rejects([&]() { parse_holding_reply(wrong_count, 4); });
  auto wrong_function = reply;
  wrong_function[0] = 4;
  rejects([&]() { parse_holding_reply(wrong_function, 4); });
}

void ultrasonic_decode_tests()
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto valid = decode_ultrasonic(25, 0.01F, 3.5F);
  require(
    valid.status == "ok" && diagnostic_level(valid) == DiagnosticStatus::OK &&
      std::fabs(valid.range_m - 0.025F) < 1.0e-6F,
    "Valid distance decode failed");
  const auto no_target = decode_ultrasonic(0xFFFD, 0.01F, 3.5F);
  require(
    std::isinf(no_target.range_m) && diagnostic_level(no_target) == DiagnosticStatus::OK &&
      no_target.status == "no_target",
    "No-target decode failed");
  require(
    decode_ultrasonic(0xFFFE, 0.01F, 3.5F).status == "interference", "Interference decode failed");
  require(
    diagnostic_level(decode_ultrasonic(0xFFFF, 0.01F, 3.5F)) == DiagnosticStatus::ERROR,
    "Timeout decode failed");
  require(
    decode_ultrasonic(0xEEEE, 0.01F, 3.5F).status == "checksum_error", "Checksum decode failed");
  require(decode_ultrasonic(0, 0.01F, 3.5F).status == "out_of_range", "Out-of-range decode failed");
}

void ultrasonic_range_message_tests()
{
  constexpr float field_of_view_rad = 0.6981317008F;
  const std::vector<std::string> frame_ids{
    "ultrasonic_channel_1_link", "ultrasonic_channel_2_link", "ultrasonic_channel_3_link",
    "ultrasonic_channel_4_link", "ultrasonic_channel_5_link", "ultrasonic_channel_6_link",
    "ultrasonic_channel_7_link", "ultrasonic_channel_8_link"};

  validate_ultrasonic_config(frame_ids, field_of_view_rad, 0.01F, 3.5F);
  rejects([&]() { validate_ultrasonic_config({}, field_of_view_rad, 0.01F, 3.5F); });
  rejects([&]() {
    validate_ultrasonic_config(
      {"ultrasonic_channel_1_link", "", "ultrasonic_channel_3_link", "ultrasonic_channel_4_link",
       "ultrasonic_channel_5_link", "ultrasonic_channel_6_link", "ultrasonic_channel_7_link",
       "ultrasonic_channel_8_link"},
      field_of_view_rad, 0.01F, 3.5F);
  });
  rejects([&]() { validate_ultrasonic_config(frame_ids, 0.0F, 0.01F, 3.5F); });
  rejects([&]() { validate_ultrasonic_config(frame_ids, field_of_view_rad, 0.01F, 1.5F); });

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 123456789U;
  const auto message = make_range_message(
    stamp, frame_ids[0], decode_ultrasonic(500, 0.01F, 3.5F), field_of_view_rad, 0.01F, 3.5F);
  require(message.header.stamp == stamp, "Range timestamp was not preserved");
  require(message.header.frame_id == frame_ids[0], "Range frame_id was not preserved");
  require(
    message.radiation_type == sensor_msgs::msg::Range::ULTRASOUND,
    "Range radiation type must be ultrasound");
  require(
    std::fabs(message.field_of_view - field_of_view_rad) < 1.0e-6F,
    "Range field of view must be 40 degrees");
  require(std::fabs(message.min_range - 0.01F) < 1.0e-6F, "Range minimum is incorrect");
  require(std::fabs(message.max_range - 3.5F) < 1.0e-6F, "Range maximum is incorrect");
  require(std::fabs(message.range - 0.5F) < 1.0e-6F, "Range distance is incorrect");

  const auto no_target_message = make_range_message(
    stamp, frame_ids[1], decode_ultrasonic(0xFFFD, 0.01F, 3.5F), field_of_view_rad, 0.01F, 3.5F);
  require(
    std::isinf(no_target_message.range) && no_target_message.range > 0.0F,
    "No-target Range value must be positive infinity");
}

int main()
{
  try {
    std::cout << "parameters and encoding\n";
    parameter_and_encoding_tests();
    std::cout << "replies\n";
    reply_tests();
    std::cout << "ultrasonic decode\n";
    ultrasonic_decode_tests();
    std::cout << "ultrasonic Range message\n";
    ultrasonic_range_message_tests();
    std::cout << "PASS: Modbus request/reply encoding and ultrasonic decoding\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
