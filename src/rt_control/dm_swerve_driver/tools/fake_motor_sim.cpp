#include <csignal>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dm_swerve_driver/fake_motor_model.hpp"
#include "dm_swerve_driver/socketcan_interface.hpp"

namespace {

using dm_swerve_driver::FakeMotorConfig;
using dm_swerve_driver::FakeMotorModel;
using dm_swerve_driver::MotorLimits;
using dm_swerve_driver::SocketCanInterface;
using dm_swerve_driver::SocketCanOptions;

volatile std::sig_atomic_t stop_requested{0};

struct ProgramOptions {
  std::string interface_name{"vcan0"};
  std::chrono::microseconds response_delay{0};
  std::chrono::duration<double> duration{0.0};
  std::size_t drop_every{0U};
  std::vector<std::pair<std::uint16_t, std::uint16_t>> motors;
};

void handle_signal(int) noexcept
{
  stop_requested = 1;
}

[[nodiscard]] unsigned long parse_unsigned(const std::string & text, const std::string & option)
{
  std::size_t consumed{0U};
  const unsigned long value = std::stoul(text, &consumed, 0);
  if (consumed != text.size()) {
    throw std::invalid_argument{option + " requires an unsigned integer"};
  }
  return value;
}

[[nodiscard]] std::pair<std::uint16_t, std::uint16_t> parse_motor_pair(
  const std::string & text)
{
  const auto separator = text.find(':');
  if (separator == std::string::npos) {
    throw std::invalid_argument{"--motor expects ESC_ID:MST_ID"};
  }
  const auto esc = parse_unsigned(text.substr(0U, separator), "--motor");
  const auto mst = parse_unsigned(text.substr(separator + 1U), "--motor");
  if (esc > 0x7FFUL || mst > 0x7FFUL) {
    throw std::invalid_argument{"--motor identifiers must fit in 11 bits"};
  }
  return {
    static_cast<std::uint16_t>(esc),
    static_cast<std::uint16_t>(mst)};
}

void print_usage(std::ostream & output)
{
  output <<
    "Usage: fake_motor_sim [options]\n"
    "  --interface NAME       SocketCAN interface (default: vcan0)\n"
    "  --motor ESC:MST        Add a motor; repeat for multiple motors\n"
    "  --response-delay-us N  Delay each response by N microseconds\n"
    "  --drop-every N         Drop every Nth response (0 disables)\n"
    "  --duration-s SECONDS   Exit after a duration (0 runs until signal)\n"
    "  --help                 Show this help\n";
}

[[nodiscard]] ProgramOptions parse_arguments(int argc, char ** argv)
{
  ProgramOptions options;
  for (int index{1}; index < argc; ++index) {
    const std::string argument{argv[index]};
    const auto require_value = [&](const std::string & option) -> std::string {
        if (++index >= argc) {
          throw std::invalid_argument{option + " requires a value"};
        }
        return argv[index];
      };

    if (argument == "--help") {
      print_usage(std::cout);
      std::exit(0);
    } else if (argument == "--interface") {
      options.interface_name = require_value(argument);
    } else if (argument == "--motor") {
      options.motors.push_back(parse_motor_pair(require_value(argument)));
    } else if (argument == "--response-delay-us") {
      const auto value = parse_unsigned(require_value(argument), argument);
      if (value > static_cast<unsigned long>(std::numeric_limits<std::int64_t>::max())) {
        throw std::out_of_range{"--response-delay-us is too large"};
      }
      options.response_delay = std::chrono::microseconds{static_cast<std::int64_t>(value)};
    } else if (argument == "--drop-every") {
      options.drop_every = static_cast<std::size_t>(parse_unsigned(require_value(argument), argument));
    } else if (argument == "--duration-s") {
      const std::string value = require_value(argument);
      options.duration = std::chrono::duration<double>{std::stod(value)};
      if (options.duration < std::chrono::duration<double>::zero()) {
        throw std::invalid_argument{"--duration-s cannot be negative"};
      }
    } else {
      throw std::invalid_argument{"unknown option: " + argument};
    }
  }

  if (options.motors.empty()) {
    for (std::uint16_t index{0U}; index < 8U; ++index) {
      options.motors.emplace_back(
        static_cast<std::uint16_t>(index + 1U),
        static_cast<std::uint16_t>(0x11U + index));
    }
  }
  return options;
}

[[nodiscard]] std::vector<FakeMotorModel> make_motors(const ProgramOptions & options)
{
  std::vector<FakeMotorModel> motors;
  motors.reserve(options.motors.size());
  std::transform(
    options.motors.begin(), options.motors.end(), std::back_inserter(motors),
    [](const auto & ids) {
      return FakeMotorModel{FakeMotorConfig{
          ids.first, ids.second, MotorLimits{12.5, 30.0, 10.0},
          std::chrono::milliseconds{30}}};
    });
  return motors;
}

int run(const ProgramOptions & options)
{
  auto motors = make_motors(options);
  std::vector<std::uint16_t> receive_ids{dm_swerve_driver::kRegisterCanId};
  receive_ids.reserve(motors.size() + 1U);
  std::transform(
    motors.begin(), motors.end(), std::back_inserter(receive_ids),
    [](const FakeMotorModel & motor) {return motor.esc_id();});

  SocketCanInterface can{SocketCanOptions{
      options.interface_name, receive_ids, std::chrono::milliseconds{5}, false}};
  const auto started = std::chrono::steady_clock::now();
  auto previous_update = started;
  std::size_t response_count{0U};

  while (stop_requested == 0) {
    const auto now = std::chrono::steady_clock::now();
    if (options.duration > std::chrono::duration<double>::zero() &&
      now - started >= options.duration)
    {
      break;
    }
    const auto requests = can.collect(32U, now + std::chrono::milliseconds{100});
    const auto update_time = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration<double>{update_time - previous_update};
    previous_update = update_time;

    for (const auto & request : requests) {
      for (auto & motor : motors) {
        const std::optional<dm_swerve_driver::CanFrame> response =
          motor.handle_frame(request.frame, elapsed);
        if (!response.has_value()) {
          continue;
        }
        ++response_count;
        if (options.drop_every != 0U && response_count % options.drop_every == 0U) {
          break;
        }
        if (options.response_delay > std::chrono::microseconds::zero()) {
          std::this_thread::sleep_for(options.response_delay);
        }
        can.write_batch({*response});
        break;
      }
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  try {
    return run(parse_arguments(argc, argv));
  } catch (const std::exception & error) {
    std::cerr << "fake_motor_sim: " << error.what() << '\n';
    return 2;
  }
}
