// Startup self-check stage A for the DaMiao head: one disable command per motor, one JSON line
// per motor on stdout: {"can_id":N,"answered":true|false,"status":ERR}. Exit 2 when the CAN
// socket cannot be opened. Usage: damiao_preflight <can_interface> <timeout_ms> <can_id>:<master_id>...
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "hw_can_damiao/socketcan.hpp"
#include "hw_can_damiao/status_query.hpp"

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "usage: damiao_preflight <can_interface> <timeout_ms> <can_id>:<master_id>...\n";
    return EXIT_FAILURE;
  }
  struct Motor
  {
    std::uint16_t can_id;
    std::uint16_t master_id;
  };
  std::vector<Motor> motors;
  std::vector<std::uint16_t> receive_ids;
  try {
    for (int index{3}; index < argc; ++index) {
      const std::string pair{argv[index]};
      const auto colon = pair.find(':');
      motors.push_back(
        {static_cast<std::uint16_t>(std::stoul(pair.substr(0, colon))),
         static_cast<std::uint16_t>(std::stoul(pair.substr(colon + 1U)))});
      receive_ids.push_back(motors.back().master_id);
    }
    const std::chrono::milliseconds timeout{std::stoi(argv[2])};
    hw_can_damiao::SocketCan socket;
    try {
      socket.open(argv[1], receive_ids);
    } catch (const std::exception & error) {
      std::cerr << "cannot open " << argv[1] << ": " << error.what() << '\n';
      return 2;
    }
    for (const auto & motor : motors) {
      const auto reply =
        hw_can_damiao::query_status(socket, motor.can_id, motor.master_id, timeout);
      std::cout << "{\"can_id\":" << motor.can_id
                << ",\"answered\":" << (reply.answered ? "true" : "false")
                << ",\"status\":" << static_cast<int>(reply.status) << "}\n";
    }
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "damiao_preflight: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
