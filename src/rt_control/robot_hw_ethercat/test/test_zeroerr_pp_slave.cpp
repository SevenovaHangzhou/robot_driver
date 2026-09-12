#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <functional>
#include <unordered_map>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include "pluginlib/class_loader.hpp"

#include "robot_hw_ethercat/zeroerr_pp_slave.hpp"

namespace
{
struct TemporaryProfile
{
  std::array<char, 30> path{};
  TemporaryProfile()
  {
    const std::string pattern = "/tmp/zeroerr-pp-test-XXXXXX";
    std::copy(pattern.begin(), pattern.end(), path.begin());
    const int descriptor = mkstemp(path.data());
    if (descriptor < 0) {throw std::runtime_error("Cannot create test fixture");}
    close(descriptor);
  }
  ~TemporaryProfile() {std::filesystem::remove(path.data());}
};
}  // namespace

class PpSlaveTest : public testing::Test
{
protected:
  robot_hw_ethercat::ZeroErrPpSlave slave;
  std::vector<double> states = std::vector<double>(6, 0.0);
  std::vector<double> commands{0.04, 5.0, 0.0, 1.0, 0.0};
  std::array<std::array<uint8_t, 4>, 8> pdo{};
  std::unordered_map<std::string, std::string> parameters{
    {"slave_config", PP_FIXTURE},
    {"command_interface/position", "0"}, {"command_interface/max_effort", "1"},
    {"command_interface/pp_sequence", "2"}, {"command_interface/pp_halt", "3"},
    {"command_interface/control_word", "4"},
    {"state_interface/position", "0"}, {"state_interface/velocity", "1"},
    {"state_interface/effort", "2"}, {"state_interface/status_word", "3"},
    {"state_interface/pp_sequence", "4"}, {"state_interface/pp_state", "5"}};

  void SetUp() override
  {
    ASSERT_TRUE(slave.setupSlave(parameters, &states, &commands));
    slave.set_state_is_operational(true);
    EC_WRITE_U16(pdo[3].data(), 0x0027);
    EC_WRITE_S8(pdo[4].data(), 1);
    EC_WRITE_S32(pdo[5].data(), 100);
  }

  void cycle(bool valid = true)
  {
    for (size_t i = 0; i < pdo.size(); ++i) {slave.processData(i, pdo[i].data());}
    slave.onPdoCycleStart(valid);
    for (size_t i = 0; i < pdo.size(); ++i) {slave.processData(i, pdo[i].data());}
    slave.onPdoCycleSent();
  }
};

TEST_F(PpSlaveTest, PreloadsBeforeReadyAndPreservesControlWordOwnership)
{
  EXPECT_FALSE(slave.initialized());
  cycle();
  EXPECT_TRUE(slave.initialized());
  EXPECT_EQ(EC_READ_S32(pdo[2].data()), 100);
  commands[4] = 0x000f;
  cycle();
  commands[2] = 1;
  commands[3] = 0;
  cycle();
  EXPECT_EQ(EC_READ_U16(pdo[1].data()), 100);
  EXPECT_EQ(EC_READ_S32(pdo[2].data()), 4100);
  EXPECT_EQ(EC_READ_U16(pdo[0].data()) & 0x0010, 0);
  cycle();
  EXPECT_EQ(EC_READ_U16(pdo[0].data()) & 0x0010, 0x0010);
  EXPECT_EQ(commands[4], 0x000f);
  commands[4] = 0x0006;
  cycle();
  EXPECT_EQ(EC_READ_U16(pdo[0].data()), 0x0006);
}

TEST_F(PpSlaveTest, IncompleteWorkingCounterInvalidatesReadiness)
{
  cycle();
  EXPECT_TRUE(slave.initialized());
  cycle(false);
  EXPECT_FALSE(slave.initialized());
  EXPECT_EQ(states[5], 0.0);
}

TEST_F(PpSlaveTest, MissingInterfaceAndNonfiniteControlWordFailClosed)
{
  robot_hw_ethercat::ZeroErrPpSlave incomplete;
  parameters.erase("command_interface/pp_halt");
  EXPECT_FALSE(incomplete.setupSlave(parameters, &states, &commands));
  cycle();
  commands[4] = std::numeric_limits<double>::quiet_NaN();
  cycle();
  EXPECT_EQ(EC_READ_U16(pdo[0].data()), 0);
}

TEST_F(PpSlaveTest, RejectsUnverifiedAndMalformedDeviceProfiles)
{
  TemporaryProfile file;
  const auto original = YAML::LoadFile(PP_FIXTURE);
  const std::vector<std::function<void(YAML::Node)>> mutations{
    [](auto c) {c["pp"]["verified"] = false;},
    [](auto c) {c["pp"]["counts_per_metre"] = 0.0;},
    [](auto c) {c["pp"]["velocity_metres_per_unit"] = 0.0;},
    [](auto c) {c["pp"]["effort_newtons_per_unit"] = 0.0;},
    [](auto c) {c["rpdo"][0]["channels"][0]["command_interface"] = "control_word";},
    [](auto c) {c["rpdo"][0]["channels"][1]["type"] = "uint16";},
    [](auto c) {c["rpdo"][0]["channels"][2]["index"] = 0x6072;},
    [](auto c) {c["tpdo"][0]["channels"][0]["factor"] = 2.0;},
    [](auto c) {c["tpdo"][0]["channels"].remove(5);},
    [](auto c) {c["tpdo"][0]["channels"][5]["type"] = "uint16";},
    [](auto c) {c["tpdo"][0]["channels"][5]["sub_index"] = 1;},
    [](auto c) {c["tpdo"][0]["channels"][5]["state_interface"] = "position";},
    [](auto c) {
      auto channels = c["tpdo"][0]["channels"];
      auto first = YAML::Clone(channels[0]);
      channels[0] = YAML::Clone(channels[5]);
      channels[5] = first;
    },
    [](auto c) {c["tpdo"][0]["channels"].push_back(YAML::Clone(c["tpdo"][0]["channels"][5]));},
    [](auto c) {c["pp"]["max_torque_permille"] = -1;},
    [](auto c) {c["sdo"][0]["value"] = 8;},
    [](auto c) {c["sdo"][1]["value"] = 0;},
    [](auto c) {c["sdo"][1]["index"] = 0x6040;},
    [](auto c) {c["sdo"].remove(4);}};
  parameters["slave_config"] = file.path.data();
  for (const auto & mutate : mutations) {
    auto config = YAML::Clone(original);
    mutate(config);
    std::ofstream(file.path.data()) << config;
    robot_hw_ethercat::ZeroErrPpSlave candidate;
    EXPECT_FALSE(candidate.setupSlave(parameters, &states, &commands));
  }
}

TEST_F(PpSlaveTest, SignedTorqueWithPaddingPreservesPositiveWireValue)
{
  auto config = YAML::LoadFile(PP_FIXTURE);
  config["rpdo"][0]["channels"][1]["type"] = "int16";
  config["tpdo"][0]["channels"] = YAML::Load(
    "[{index: 0x6041, sub_index: 0, type: uint16},"
    " {index: 0x6061, sub_index: 0, type: int8},"
    " {index: 0x6064, sub_index: 0, type: int32},"
    " {index: 0x606c, sub_index: 0, type: int32},"
    " {index: 0x6077, sub_index: 0, type: int16},"
    " {index: 0, sub_index: 0, type: uint8}]");
  config["pp"]["max_torque_permille"] = 32767;
  config["pp"]["max_force"] = 32767.0;
  config["pp"]["permille_per_newton"] = 1.0;
  TemporaryProfile file;
  std::ofstream(file.path.data()) << config;
  parameters["slave_config"] = file.path.data();
  robot_hw_ethercat::ZeroErrPpSlave candidate;
  ASSERT_TRUE(candidate.setupSlave(parameters, &states, &commands));
  const auto * syncs = candidate.syncs();
  ASSERT_EQ(syncs[3].n_pdos, 1U);
  ASSERT_EQ(syncs[3].pdos[0].n_entries, 6U);
  unsigned int bits = 0;
  for (size_t i = 0; i < syncs[3].pdos[0].n_entries; ++i) {
    bits += syncs[3].pdos[0].entries[i].bit_length;
  }
  EXPECT_EQ(bits, 112U);
  EXPECT_EQ(syncs[3].pdos[0].entries[5].index, 0U);
  candidate.set_state_is_operational(true);
  auto candidate_cycle = [&]() {
      for (size_t i = 0; i < pdo.size(); ++i) {candidate.processData(i, pdo[i].data());}
      candidate.onPdoCycleStart(true);
      for (size_t i = 0; i < pdo.size(); ++i) {candidate.processData(i, pdo[i].data());}
      candidate.onPdoCycleSent();
    };
  candidate_cycle();
  commands[4] = 0x000f;
  candidate_cycle();
  commands[1] = 32767.0;
  commands[2] = 1.0;
  commands[3] = 0.0;
  candidate_cycle();
  EXPECT_EQ(EC_READ_S16(pdo[1].data()), 32767);
  EXPECT_EQ(EC_READ_S32(pdo[2].data()), 4100);
  EXPECT_DOUBLE_EQ(states[0], 0.0);
}

TEST_F(PpSlaveTest, RejectsTorqueCeilingOutsidePositiveSignedRange)
{
  auto config = YAML::LoadFile(PP_FIXTURE);
  config["pp"]["max_torque_permille"] = 32768;
  TemporaryProfile file;
  std::ofstream(file.path.data()) << config;
  parameters["slave_config"] = file.path.data();
  robot_hw_ethercat::ZeroErrPpSlave candidate;
  EXPECT_FALSE(candidate.setupSlave(parameters, &states, &commands));
}

TEST_F(PpSlaveTest, InvalidatesStateBeforeControllerUpdateAndKeepsInstancesIndependent)
{
  cycle();
  commands[4] = 0x000f;
  cycle();
  EXPECT_EQ(states[5], 1.0);
  robot_hw_ethercat::ZeroErrPpSlave other;
  std::vector<double> other_states(6, 0.0);
  std::vector<double> other_commands(5, 0.0);
  ASSERT_TRUE(other.setupSlave(parameters, &other_states, &other_commands));
  other.set_state_is_operational(false);
  EXPECT_EQ(states[5], 1.0);
  slave.onPdoCycleRead(false);
  EXPECT_EQ(states[5], 0.0);
  EXPECT_FALSE(slave.initialized());
}

TEST(PpPlugin, LoadsTwoIndependentInstancesThroughTheInstalledRegistry)
{
  pluginlib::ClassLoader<ethercat_interface::EcSlave> loader(
    "ethercat_interface", "ethercat_interface::EcSlave");
  const auto left = loader.createSharedInstance("robot_hw_ethercat/ZeroErrPpSlave");
  const auto right = loader.createSharedInstance("robot_hw_ethercat/ZeroErrPpSlave");
  ASSERT_TRUE(left);
  ASSERT_TRUE(right);
  EXPECT_NE(left.get(), right.get());
}
