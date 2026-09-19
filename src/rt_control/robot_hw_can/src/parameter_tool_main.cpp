#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "robot_hw_can/parameter_tool.hpp"

int main(int argc, char ** argv)
{
  try {
    std::vector<std::string> arguments;
    arguments.reserve(argc > 1 ? static_cast<std::size_t>(argc - 1) : 0U);
    for (int index{1}; index < argc; ++index) {
      arguments.emplace_back(argv[index]);
    }
    const auto request = robot_hw_can::parse_parameter_request(arguments);
    robot_hw_can::execute_parameter_write(request);
    std::cout << "DaMiao trapezoidal profile written and verified"
              << (request.save_to_flash ? "; flash save acknowledged\n" : "; volatile only\n");
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "DaMiao parameter write failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
