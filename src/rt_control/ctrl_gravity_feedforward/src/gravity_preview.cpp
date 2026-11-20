#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "ctrl_gravity_feedforward/gravity_model.hpp"

// Offline only: URDF path, base, tip, gravity xyz; stdin lines contain chain angles (rad).
int main(int argc, char ** argv)
{
  try {
    if (argc != 7) {
      throw std::invalid_argument("usage: gravity_preview URDF BASE TIP GX GY GZ < positions.txt");
    }
    std::ifstream file(argv[1]);
    if (!file) {
      throw std::invalid_argument("cannot read URDF");
    }
    const std::string xml{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    ctrl_gravity_feedforward::GravityModel model(
      xml, argv[2], argv[3],
      std::array<double, 3>{std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6])});
    std::vector<double> positions(model.joints().size());
    std::cout << std::setprecision(17);
    while (std::cin >> positions[0]) {
      for (std::size_t index = 1; index < positions.size(); ++index) {
        if (!(std::cin >> positions[index])) {
          throw std::invalid_argument("incomplete joint position vector");
        }
      }
      if (!model.compute(positions)) {
        throw std::invalid_argument("invalid gravity input or result");
      }
      for (std::size_t index = 0; index < positions.size(); ++index) {
        std::cout << (index ? " " : "") << model.torques()[index];
      }
      std::cout << '\n';
    }
    if (!std::cin.eof()) {
      throw std::invalid_argument("invalid joint position input");
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
