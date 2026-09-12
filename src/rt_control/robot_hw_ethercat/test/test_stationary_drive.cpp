#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <unistd.h>

#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

class StationaryDrive : public testing::TestWithParam<int> {};

TEST_P(StationaryDrive, RawPreloadIsHeldWithoutAPositionCommandAndNeverTriggersPp)
{
  std::array<char, 40> path{};
  const std::string pattern = "/tmp/stationary-drive-XXXXXX";
  std::copy(pattern.begin(), pattern.end(), path.begin());
  const auto fd = mkstemp(path.data());
  ASSERT_GE(fd, 0);
  close(fd);
  struct Cleanup {const char * path; ~Cleanup() {std::filesystem::remove(path);}} cleanup{path.data()};
  const auto yaml = YAML::Load(R"(
vendor_id: 1
product_id: 2
assign_activate: 0
auto_fault_reset: false
auto_state_transitions: false
rpdo:
  - index: 0x1600
    channels:
      - {index: 0x607a, sub_index: 0, type: int32, default: .nan}
      - {index: 0x6040, sub_index: 0, type: uint16, command_interface: control_word, default: 0}
tpdo:
  - index: 0x1a00
    channels:
      - {index: 0x6064, sub_index: 0, type: int32, state_interface: position_raw}
      - {index: 0x60fd, sub_index: 0, type: uint32, state_interface: digital_inputs}
      - {index: 0x6041, sub_index: 0, type: uint16, state_interface: status_word}
)");
  std::ofstream(path.data()) << yaml;
  ethercat_generic_plugins::EcCiA402Drive drive;
  std::vector<double> states(3, 0.0), commands(1, 0.0);
  const std::unordered_map<std::string, std::string> parameters{
    {"slave_config", path.data()}, {"mode_of_operation", std::to_string(GetParam())},
    {"command_interface/control_word", "0"}, {"state_interface/position_raw", "0"},
    {"state_interface/digital_inputs", "1"}, {"state_interface/status_word", "2"}};
  ASSERT_TRUE(drive.setupSlave(parameters, &states, &commands));
  drive.set_state_is_operational(true);
  std::array<std::array<uint8_t, 4>, 5> pdo{};
  constexpr int32_t captured = 123456789;
  EC_WRITE_S32(pdo[2].data(), captured);
  EC_WRITE_U16(pdo[4].data(), 0x40);
  auto cycle = [&] {
      for (size_t i = 0; i < pdo.size(); ++i) {drive.processData(i, pdo[i].data());}
    };
  EXPECT_FALSE(drive.initialized());
  cycle();
  cycle();
  EXPECT_EQ(EC_READ_S32(pdo[0].data()), captured);
  EXPECT_FALSE(drive.initialized());
  commands[0] = 0x0f;
  cycle();
  EXPECT_EQ(EC_READ_U16(pdo[1].data()), 0x07);  // enable blocked before sent acknowledgment
  drive.onPdoCycleSent();
  EXPECT_TRUE(drive.initialized());
  cycle();
  EXPECT_EQ(EC_READ_U16(pdo[1].data()), 0x0f);
  EC_WRITE_U16(pdo[4].data(), 0x27);
  cycle();
  for (int i = 1; i <= 20; ++i) {
    EC_WRITE_S32(pdo[2].data(), captured + i);
    cycle();
    drive.onPdoCycleSent();
    EXPECT_EQ(EC_READ_S32(pdo[0].data()), captured);
    EXPECT_EQ(EC_READ_U16(pdo[1].data()) & 0x70, 0);  // no new-setpoint/relative bits
  }
  commands[0] = 0;
  EC_WRITE_U16(pdo[4].data(), 0x40);
  cycle();
  EXPECT_FALSE(drive.initialized());
  cycle();
  EXPECT_EQ(EC_READ_U16(pdo[1].data()), 0);
  EXPECT_EQ(EC_READ_S32(pdo[0].data()), captured + 20);
}

INSTANTIATE_TEST_SUITE_P(CspAndPp, StationaryDrive, testing::Values(8, 1));
