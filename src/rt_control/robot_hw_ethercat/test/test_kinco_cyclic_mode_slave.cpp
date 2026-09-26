#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <unistd.h>

#include "pluginlib/class_loader.hpp"
#include "robot_hw_ethercat/kinco_cyclic_mode_slave.hpp"

namespace
{
struct TemporaryProfile
{
  std::array<char, 36> path{};
  TemporaryProfile()
  {
    const std::string pattern = "/tmp/kinco-mode-test-XXXXXX";
    std::copy(pattern.begin(), pattern.end(), path.begin());
    const int descriptor = mkstemp(path.data());
    if (descriptor < 0) {
      throw std::runtime_error("cannot create Kinco test fixture");
    }
    close(descriptor);
  }
  ~TemporaryProfile() {std::filesystem::remove(path.data());}
};

struct Rig
{
  explicit Rig(const std::string & profile = KINCO_FIXTURE, int startup_mode = 9)
  : profile_path(profile), states(13, 0.0), commands{
      0.25, std::numeric_limits<double>::quiet_NaN(), static_cast<double>(startup_mode), 0x000f,
      0.0, 0.0}
  {
    parameters = {
      {"slave_config", profile_path}, {"allow_mock_profile", "true"},
      {"mode_of_operation", std::to_string(startup_mode)},
      {"command_interface/position", "0"}, {"command_interface/velocity", "1"},
      {"command_interface/mode_of_operation", "2"}, {"command_interface/control_word", "3"},
      {"state_interface/position", "0"}, {"state_interface/velocity", "1"},
      {"state_interface/mode_of_operation_display", "2"},
      {"state_interface/status_word", "3"}, {"state_interface/feedback_age_ms", "4"},
      {"state_interface/mode_switch_ready", "5"}, {"state_interface/mode_request_ack", "6"},
      {"state_interface/command_fresh", "7"}, {"state_interface/mode_request_error", "8"},
      {"command_interface/write_sequence", "4"}, {"command_interface/write_mask", "5"},
      {"state_interface/feedback_sequence", "9"}, {"state_interface/sent_sequence", "10"},
      {"state_interface/feedback_sequence_at_send", "11"}, {"state_interface/sent_velocity", "12"}};
  }

  void setup()
  {
    ASSERT_TRUE(slave.setupSlave(parameters, &states, &commands));
    const auto config = YAML::LoadFile(profile_path);
    size_t domain = 0;
    for (const char * direction : {"rpdo", "tpdo"}) {
      for (const auto & pdo : config[direction]) {
        for (const auto & channel : pdo["channels"]) {
          index[channel["index"].as<unsigned int>()] = domain++;
        }
      }
    }
    ASSERT_EQ(domain, pdo.size());
    slave.set_state_is_operational(true);
  }

  void feedback(
    int8_t mode = 9, int32_t position = 350, int32_t velocity = 0,
    uint16_t status = 0x0027, bool complete = true)
  {
    EC_WRITE_U16(bytes(0x6041), status);
    EC_WRITE_S32(bytes(0x6064), position);
    EC_WRITE_S32(bytes(0x606c), velocity);
    EC_WRITE_S8(bytes(0x6061), mode);
    slave.onPdoCycleRead(complete);
    for (size_t current = 4; current < pdo.size(); ++current) {
      slave.processData(current, pdo[current].data());
    }
  }

  void send(bool complete = true)
  {
    slave.onPdoCycleStart(complete);
    for (size_t current = 0; current < 4; ++current) {
      slave.processData(current, pdo[current].data());
    }
    slave.onPdoCycleSent();
  }

  void cycle(
    int8_t mode = 9, int32_t position = 350, int32_t velocity = 0,
    uint16_t status = 0x0027, bool complete = true)
  {
    feedback(mode, position, velocity, status, complete);
    send(complete);
  }

  uint8_t * bytes(unsigned int object) {return pdo.at(index.at(object)).data();}

  std::string profile_path;
  robot_hw_ethercat::KincoCyclicModeSlave slave;
  std::vector<double> states;
  std::vector<double> commands;
  std::array<std::array<uint8_t, 8>, 8> pdo{};
  std::unordered_map<unsigned int, size_t> index;
  std::unordered_map<std::string, std::string> parameters;
};
}  // namespace

TEST(KincoCyclicModeSlave, StartupSeedsActualRawPositionInCsvBeforeReportingInitialized)
{
  Rig rig;
  rig.setup();
  EXPECT_FALSE(rig.slave.initialized());
  rig.cycle();
  EXPECT_TRUE(rig.slave.initialized());
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x607a)), 350);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
  EXPECT_EQ(EC_READ_S8(rig.bytes(0x6060)), 9);
  EXPECT_EQ(EC_READ_U16(rig.bytes(0x6040)), 0x0007);
  EXPECT_NEAR(rig.states[0], 0.25, 1e-12);
  EXPECT_DOUBLE_EQ(rig.states[1], 0.0);
  EXPECT_TRUE(std::isnan(rig.commands[1]));

  rig.commands[1] = 1.5;
  rig.cycle();
  EXPECT_EQ(EC_READ_U16(rig.bytes(0x6040)), 0x000f);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 150);
  EXPECT_DOUBLE_EQ(rig.states[6], 1.0);
  EXPECT_DOUBLE_EQ(rig.states[7], 1.0);
}

TEST(KincoCyclicModeSlave, CsvToCspRequiresStationaryExactPreseedSentBeforeModeRequest)
{
  Rig rig;
  rig.setup();
  rig.cycle();
  rig.commands[0] = 0.25;
  rig.commands[1] = 0.0;
  rig.commands[2] = 8.0;

  rig.cycle(9, 350, 3);
  EXPECT_EQ(EC_READ_S8(rig.bytes(0x6060)), 9);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x607a)), 350);
  EXPECT_DOUBLE_EQ(rig.states[5], 0.0);

  rig.cycle(9, 350, 0);
  EXPECT_EQ(EC_READ_S8(rig.bytes(0x6060)), 9);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x607a)), 350);
  EXPECT_DOUBLE_EQ(rig.states[5], 1.0);

  rig.cycle(9, 350, 0);
  EXPECT_EQ(EC_READ_S8(rig.bytes(0x6060)), 8);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x607a)), 350);
  EXPECT_DOUBLE_EQ(rig.states[6], 0.0);

  rig.cycle(9, 351, 0);
  EXPECT_EQ(EC_READ_S8(rig.bytes(0x6060)), 8);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x607a)), 350);
  EXPECT_DOUBLE_EQ(rig.states[6], 0.0);

  rig.cycle(8, 351, 0);
  EXPECT_DOUBLE_EQ(rig.states[6], 1.0);
  rig.commands[0] = 0.35;
  rig.cycle(8, 350, 0);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x607a)), 450);
  EXPECT_DOUBLE_EQ(rig.states[7], 1.0);
}

TEST(KincoCyclicModeSlave, CspToCsvSendsZeroDiscardsStaleAndRequiresPostAckCommand)
{
  Rig rig(KINCO_FIXTURE, 8);
  rig.setup();
  rig.cycle(8);
  rig.commands[0] = 0.30;
  rig.commands[1] = 2.0;
  rig.commands[2] = 9.0;

  rig.cycle(8);
  EXPECT_EQ(EC_READ_S8(rig.bytes(0x6060)), 9);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
  EXPECT_TRUE(std::isnan(rig.commands[1]));
  EXPECT_DOUBLE_EQ(rig.states[6], 0.0);

  rig.commands[1] = 3.0;
  rig.cycle(8);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
  EXPECT_TRUE(std::isnan(rig.commands[1]));

  rig.commands[1] = 3.0;
  rig.cycle(9);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
  EXPECT_TRUE(std::isnan(rig.commands[1]));
  EXPECT_DOUBLE_EQ(rig.states[6], 1.0);
  EXPECT_DOUBLE_EQ(rig.states[7], 0.0);

  rig.commands[1] = 3.0;
  rig.cycle(9);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
  EXPECT_TRUE(std::isnan(rig.commands[1]));
  rig.commands[1] = -1.5;
  rig.cycle(9);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), -150);
  EXPECT_DOUBLE_EQ(rig.states[7], 1.0);
}

TEST(KincoCyclicModeSlave, RejectsInvalidModesCommandsAndOutOfRangeValues)
{
  Rig rig;
  rig.setup();
  rig.cycle();
  rig.commands[2] = 7.0;
  rig.commands[1] = 1.0;
  rig.cycle();
  EXPECT_EQ(EC_READ_S8(rig.bytes(0x6060)), 9);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
  EXPECT_DOUBLE_EQ(rig.states[8], 1.0);

  rig.commands[2] = 9.0;
  rig.commands[1] = 4.01;
  rig.cycle();
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
  rig.commands[1] = std::numeric_limits<double>::quiet_NaN();
  rig.commands[3] = std::numeric_limits<double>::infinity();
  rig.cycle();
  EXPECT_EQ(EC_READ_U16(rig.bytes(0x6040)), 0);

  Rig csp(KINCO_FIXTURE, 8);
  csp.setup();
  csp.cycle(8);
  csp.commands[0] = 2.01;
  csp.cycle(8);
  EXPECT_EQ(EC_READ_S32(csp.bytes(0x607a)), 350);
  EXPECT_DOUBLE_EQ(csp.states[8], 1.0);
}

TEST(KincoCyclicModeSlave, IncompleteDataAndLifecycleLossClearReadinessAndOutputs)
{
  Rig rig;
  rig.setup();
  rig.cycle();
  rig.commands[1] = 1.0;
  rig.cycle();
  ASSERT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 100);

  rig.cycle(9, 350, 0, 0x0027, false);
  EXPECT_FALSE(rig.slave.initialized());
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
  EXPECT_EQ(EC_READ_U16(rig.bytes(0x6040)), 0);
  EXPECT_TRUE(std::isinf(rig.states[4]));
  EXPECT_DOUBLE_EQ(rig.states[5], 0.0);
  EXPECT_DOUBLE_EQ(rig.states[6], 0.0);
  EXPECT_DOUBLE_EQ(rig.states[8], 1.0);

  rig.slave.set_state_is_operational(false);
  EXPECT_TRUE(std::isnan(rig.commands[1]));
  rig.slave.set_state_is_operational(true);
  rig.cycle();
  EXPECT_TRUE(rig.slave.initialized());
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
}

TEST(KincoCyclicModeSlave, ChannelOrderingDoesNotChangeSameCycleModeAndPositionOutput)
{
  auto config = YAML::LoadFile(KINCO_FIXTURE);
  auto reversed = [](const YAML::Node & channels) {
      YAML::Node result(YAML::NodeType::Sequence);
      for (size_t index = channels.size(); index > 0; --index) {
        result.push_back(YAML::Clone(channels[index - 1]));
      }
      return result;
    };
  config["rpdo"][0]["channels"] = reversed(config["rpdo"][0]["channels"]);
  config["tpdo"][0]["channels"] = reversed(config["tpdo"][0]["channels"]);
  TemporaryProfile file;
  std::ofstream(file.path.data()) << config;
  Rig rig(file.path.data());
  rig.setup();
  rig.cycle();
  rig.commands[0] = 0.25;
  rig.commands[2] = 8.0;
  rig.cycle();
  rig.cycle();
  EXPECT_EQ(EC_READ_S8(rig.bytes(0x6060)), 8);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x607a)), 350);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
}

TEST(KincoCyclicModeSlave, DelayedReadbackExpiresExplicitModeTimeoutAndLatchesSafeOutput)
{
  auto config = YAML::LoadFile(KINCO_FIXTURE);
  config["kinco_cyclic_mode"]["mode_ack_timeout_seconds"] = 0.001;
  TemporaryProfile file;
  std::ofstream(file.path.data()) << config;
  Rig rig(file.path.data());
  rig.setup();
  rig.cycle();
  rig.commands[0] = 0.25;
  rig.commands[2] = 8.0;
  rig.cycle();
  rig.cycle();
  ASSERT_EQ(EC_READ_S8(rig.bytes(0x6060)), 8);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  rig.cycle(9);
  EXPECT_EQ(EC_READ_S8(rig.bytes(0x6060)), 9);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
  EXPECT_DOUBLE_EQ(rig.states[6], 0.0);
  EXPECT_DOUBLE_EQ(rig.states[8], 1.0);

  rig.commands[2] = 9.0;
  rig.commands[1] = 1.0;
  rig.cycle(9);
  EXPECT_EQ(EC_READ_S32(rig.bytes(0x60ff)), 0);
  EXPECT_DOUBLE_EQ(rig.states[8], 1.0);
}

TEST(KincoCyclicModeSlave, MockProfileRequiresExplicitOptInAndMalformedProfilesFailClosed)
{
  Rig rig;
  rig.parameters.erase("allow_mock_profile");
  EXPECT_FALSE(rig.slave.setupSlave(rig.parameters, &rig.states, &rig.commands));

  const auto original = YAML::LoadFile(KINCO_FIXTURE);
  const std::vector<std::function<void(YAML::Node)>> mutations{
    [](auto config) {config["kinco_cyclic_mode"]["verified"] = false;},
    [](auto config) {config["use_slave_pdo_defaults"] = false;},
    [](auto config) {config["kinco_cyclic_mode"]["stationary_velocity"] = 5.0;},
    [](auto config) {config["kinco_cyclic_mode"]["feedback_timeout_seconds"] = 0.0;},
    [](auto config) {config["rpdo"][0]["channels"][0]["command_interface"] = "control_word";},
    [](auto config) {config["rpdo"][0]["channels"][3]["type"] = "uint8";},
    [](auto config) {config["tpdo"][0]["channels"].remove(3);},
    [](auto config) {
      config["sdo"] = YAML::Load("[{index: 0x6060, sub_index: 0, type: int8, value: 8}]");
    }};
  for (const auto & mutate : mutations) {
    auto config = YAML::Clone(original);
    mutate(config);
    TemporaryProfile file;
    std::ofstream(file.path.data()) << config;
    Rig candidate(file.path.data());
    EXPECT_FALSE(
      candidate.slave.setupSlave(
        candidate.parameters, &candidate.states, &candidate.commands));
  }
}

TEST(KincoCyclicModePlugin, LoadsIndependentInstancesFromRegistry)
{
  pluginlib::ClassLoader<ethercat_interface::EcSlave> loader(
    "ethercat_interface", "ethercat_interface::EcSlave");
  const auto first = loader.createSharedInstance("robot_hw_ethercat/KincoCyclicModeSlave");
  const auto second = loader.createSharedInstance("robot_hw_ethercat/KincoCyclicModeSlave");
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_NE(first.get(), second.get());
}
