#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <unistd.h>

#include "hw_ethercat/tongyi_pp_slave.hpp"

namespace
{
using hw_ethercat::TongyiPpSlave;

// Wire order of the fixed layout: 7 outputs, then 11 inputs.
enum Channel : size_t {
  kControlWord,
  kTargetTorque,
  kTargetVelocity,
  kTargetPosition,
  kProfileVelocity,
  kMode,
  kMaxProfileVelocity,
  kStatusWord,
  kErrorCode,
  kCurrent,
  kVelocity,
  kActualPosition,
  kModeDisplay,
  kInputCount,
  kBridgeTemperature,
  kMotorTemperature,
  kDigitalInputs,
  kTorque,
  kChannels
};

struct TemporaryProfile
{
  std::array<char, 30> path{};
  TemporaryProfile()
  {
    const std::string pattern = "/tmp/tongyi-pp-test-XXXXXX";
    std::copy(pattern.begin(), pattern.end(), path.begin());
    const int descriptor = mkstemp(path.data());
    if (descriptor < 0) {
      throw std::runtime_error("Cannot create test fixture");
    }
    close(descriptor);
  }
  ~TemporaryProfile() { std::filesystem::remove(path.data()); }
};

std::unordered_map<std::string, std::string> interface_parameters(const std::string & file)
{
  std::unordered_map<std::string, std::string> parameters{{"slave_config", file}};
  for (size_t i = 0; i < TongyiPpSlave::kCommands.size(); ++i) {
    parameters[std::string("command_interface/") + TongyiPpSlave::kCommands[i]] = std::to_string(i);
  }
  for (size_t i = 0; i < TongyiPpSlave::kStates.size(); ++i) {
    parameters[std::string("state_interface/") + TongyiPpSlave::kStates[i]] = std::to_string(i);
  }
  return parameters;
}

constexpr uint16_t kEnabled = 0x0027;  // operation enabled
}  // namespace

class TongyiPpSlaveTest : public testing::Test
{
protected:
  TongyiPpSlave slave;
  std::vector<double> states = std::vector<double>(TongyiPpSlave::kStates.size(), 0.0);
  // position, profile_velocity, pp_sequence, pp_halt, control_word
  std::vector<double> commands{0.05, 0.01, 0.0, 0.0, 0.0};
  std::array<std::array<uint8_t, 4>, kChannels> pdo{};
  std::unordered_map<std::string, std::string> parameters = interface_parameters(TONGYI_FIXTURE);

  void SetUp() override
  {
    ASSERT_TRUE(slave.setupSlave(parameters, &states, &commands));
    slave.set_state_is_operational(true);
    feedback(500000, kEnabled, 1);
  }

  void feedback(int32_t position, uint16_t status, int8_t mode)
  {
    EC_WRITE_S32(pdo[kActualPosition].data(), position);
    EC_WRITE_U16(pdo[kStatusWord].data(), status);
    EC_WRITE_S8(pdo[kModeDisplay].data(), mode);
  }

  void cycle(bool valid = true)
  {
    slave.onPdoCycleRead(valid);
    for (size_t i = 0; i < pdo.size(); ++i) {
      slave.processData(i, pdo[i].data());
    }
    slave.onPdoCycleStart(valid);
    for (size_t i = 0; i < pdo.size(); ++i) {
      slave.processData(i, pdo[i].data());
    }
    slave.onPdoCycleSent();
  }

  double pp_state() const { return states[6]; }
};

TEST_F(TongyiPpSlaveTest, OwnsEveryFixedOutputAndPreloadsTheActualPosition)
{
  EXPECT_FALSE(slave.initialized());
  EC_WRITE_S16(pdo[kTargetTorque].data(), 77);
  EC_WRITE_S32(pdo[kTargetVelocity].data(), 77);
  cycle();
  EXPECT_TRUE(slave.initialized());
  EXPECT_EQ(EC_READ_S32(pdo[kTargetPosition].data()), 500000);
  EXPECT_EQ(EC_READ_S8(pdo[kMode].data()), 1);
  EXPECT_EQ(EC_READ_U32(pdo[kMaxProfileVelocity].data()), 3600U);
  EXPECT_EQ(EC_READ_S16(pdo[kTargetTorque].data()), 0);
  EXPECT_EQ(EC_READ_S32(pdo[kTargetVelocity].data()), 0);
  EXPECT_DOUBLE_EQ(states[0], 0.0);
}

TEST_F(TongyiPpSlaveTest, NotReadyUntilTheDriveReportsProfilePositionMode)
{
  feedback(500000, 0x0040, 0);
  cycle();
  EXPECT_FALSE(slave.initialized());
  EXPECT_EQ(EC_READ_S8(pdo[kMode].data()), 1);
  feedback(500000, 0x0040, 1);
  cycle();
  EXPECT_TRUE(slave.initialized());
}

TEST_F(TongyiPpSlaveTest, TargetLatchesItsProfileVelocityAndUsesTheSignedScale)
{
  cycle();
  commands[4] = 0x000f;
  cycle();
  commands[2] = 1;  // 50 mm up: raw decreases because the motor's positive is down
  cycle();
  EXPECT_EQ(EC_READ_S32(pdo[kTargetPosition].data()), 450000);
  EXPECT_EQ(EC_READ_U32(pdo[kProfileVelocity].data()), 1200U);
  EXPECT_EQ(EC_READ_U16(pdo[kControlWord].data()) & 0x0010, 0);
  cycle();
  EXPECT_EQ(EC_READ_U16(pdo[kControlWord].data()) & 0x0110, 0x0010);
  // A later velocity command does not change the running target's velocity.
  commands[1] = 0.02;
  cycle();
  EXPECT_EQ(EC_READ_U32(pdo[kProfileVelocity].data()), 1200U);
  EXPECT_EQ(commands[4], 0x000f);
}

TEST_F(TongyiPpSlaveTest, OutOfRangeTargetOrVelocityFaultsWithoutMoving)
{
  for (const auto & [position, velocity] : std::vector<std::pair<double, double>>{
         {0.181, 0.01}, {-0.001, 0.01}, {0.05, 0.031}, {0.05, 0.0}, {0.05, 1e-6}}) {
    TongyiPpSlave candidate;
    std::vector<double> local_states(TongyiPpSlave::kStates.size(), 0.0);
    std::vector<double> local_commands{position, velocity, 0.0, 0.0, 0.0};
    ASSERT_TRUE(candidate.setupSlave(parameters, &local_states, &local_commands));
    candidate.set_state_is_operational(true);
    auto run = [&]() {
      candidate.onPdoCycleRead(true);
      for (size_t i = 0; i < pdo.size(); ++i) {
        candidate.processData(i, pdo[i].data());
      }
      candidate.onPdoCycleStart(true);
      for (size_t i = 0; i < pdo.size(); ++i) {
        candidate.processData(i, pdo[i].data());
      }
      candidate.onPdoCycleSent();
    };
    run();
    local_commands[4] = 0x000f;
    run();
    local_commands[2] = 1;
    run();
    EXPECT_EQ(local_states[6], -1.0) << position << " " << velocity;
    EXPECT_EQ(EC_READ_S32(pdo[kTargetPosition].data()), 500000);
    EXPECT_EQ(EC_READ_U16(pdo[kControlWord].data()) & 0x0110, 0x0100);
  }
}

TEST_F(TongyiPpSlaveTest, VelocityIsThePositionDerivativeAndRawFeedbackIsExported)
{
  cycle();
  EC_WRITE_S32(pdo[kVelocity].data(), -42);
  EC_WRITE_U16(pdo[kErrorCode].data(), 0x0200);
  EC_WRITE_U16(pdo[kBridgeTemperature].data(), 351);
  EC_WRITE_U32(pdo[kDigitalInputs].data(), 5);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  feedback(499000, kEnabled, 1);  // 1 mm up
  cycle();
  EXPECT_NEAR(states[0], 0.001, 1e-12);
  EXPECT_GT(states[1], 0.0);
  EXPECT_LT(states[1], 0.001 / 0.019);
  EXPECT_EQ(states[7], -42.0);
  EXPECT_EQ(states[3], 512.0);
  EXPECT_EQ(states[10], 351.0);
  EXPECT_EQ(states[12], 5.0);
}

TEST_F(TongyiPpSlaveTest, IncompleteWorkingCounterInvalidatesReadiness)
{
  cycle();
  EXPECT_TRUE(slave.initialized());
  cycle(false);
  EXPECT_FALSE(slave.initialized());
  EXPECT_EQ(pp_state(), 0.0);
  EXPECT_TRUE(std::isnan(states[1]));
}

TEST_F(TongyiPpSlaveTest, NonfiniteControlWordFailsClosed)
{
  cycle();
  commands[4] = std::numeric_limits<double>::quiet_NaN();
  cycle();
  EXPECT_EQ(EC_READ_U16(pdo[kControlWord].data()), 0);
}

TEST_F(TongyiPpSlaveTest, RejectsMissingInterfacesAndMalformedProfiles)
{
  {
    auto incomplete = parameters;
    incomplete.erase("command_interface/profile_velocity");
    TongyiPpSlave candidate;
    EXPECT_FALSE(candidate.setupSlave(incomplete, &states, &commands));
  }
  TemporaryProfile file;
  const auto original = YAML::LoadFile(TONGYI_FIXTURE);
  const std::vector<std::function<void(YAML::Node)>> mutations{
    [](auto c) { c["pp"]["verified"] = false; },
    [](auto c) { c["use_slave_pdo_defaults"] = false; },
    [](auto c) { c["pp"]["counts_per_metre"] = 0.0; },
    [](auto c) { c["pp"]["max_velocity"] = 0.0; },
    [](auto c) { c["pp"]["profile_velocity_per_metre_per_second"] = -1.0; },
    // 30 mm/s needs 3600; a lower drive ceiling would silently clip every request.
    [](auto c) { c["pp"]["max_profile_velocity"] = 3599; },
    [](auto c) { c["rpdo"][0]["channels"][6]["command_interface"] = "velocity"; },
    [](auto c) { c["rpdo"][0]["channels"][5]["type"] = "uint8"; },
    [](auto c) { c["rpdo"][0]["channels"].remove(1); },
    [](auto c) { c["tpdo"][0]["channels"][4]["factor"] = 2.0; },
    [](auto c) { c["tpdo"][0]["channels"][3]["index"] = 0x6064; },
    [](auto c) { c["tpdo"][0]["index"] = 0x1a01; }, [](auto c) { c["sdo"][0]["value"] = 0; },
    [](auto c) { c["sdo"][1]["index"] = 0x6060; }, [](auto c) { c["sdo"].remove(1); },
    [](auto c) { c["sdo"].push_back(YAML::Clone(c["sdo"][0])); }};
  parameters["slave_config"] = file.path.data();
  for (size_t i = 0; i < mutations.size(); ++i) {
    auto config = YAML::Clone(original);
    mutations[i](config);
    std::ofstream(file.path.data()) << config;
    TongyiPpSlave candidate;
    EXPECT_FALSE(candidate.setupSlave(parameters, &states, &commands)) << "mutation " << i;
  }
}
