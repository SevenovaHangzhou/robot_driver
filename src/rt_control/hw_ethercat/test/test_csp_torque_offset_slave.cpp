#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <vector>

#include "hw_ethercat/csp_torque_offset_slave.hpp"
#include "pluginlib/class_loader.hpp"

namespace
{
struct Rig
{
  hw_ethercat::CspTorqueOffsetSlave drive;
  std::vector<double> states{0, 0, 0, 0, 0};
  std::vector<double> commands{0, 0, 900};
  std::array<std::array<uint8_t, 4>, 5> pdo{};

  bool setup(const std::string & mode = "8")
  {
    return drive.setupSlave(
      {{"slave_config", GRAVITY_FIXTURE},
       {"mode_of_operation", mode},
       {"command_interface/position", "0"},
       {"command_interface/control_word", "1"},
       {"command_interface/effort", "2"},
       {"state_interface/position", "0"},
       {"state_interface/status_word", "1"},
       {"state_interface/gravity_effort_sent", "2"},
       {"state_interface/torque_offset_permille_sent", "3"},
       {"state_interface/gravity_command_epoch", "4"}},
      &states, &commands);
  }
  void cycle(uint16_t status, bool complete = true)
  {
    drive.onPdoCycleStart(complete);
    EC_WRITE_U16(pdo[4].data(), status);
    // Use the driver's coherent read-then-write cycle ordering.
    for (size_t i = 3; i < 5; ++i) {
      drive.processData(i, pdo[i].data());
    }
    drive.onPdoCycleRead(complete);
    for (size_t i = 0; i < 3; ++i) {
      drive.processData(i, pdo[i].data());
    }
    drive.onPdoCycleSent();
  }
  void enable()
  {
    drive.set_state_is_operational(true);
    cycle(0x40);
    commands[1] = 0xf;
    cycle(0x27);
  }
  int16_t offset() const { return EC_READ_S16(pdo[2].data()); }
};

TEST(CspTorqueOffset, ConvertsOutputNmAndMirrorsQuantizedSentValue)
{
  Rig rig;
  ASSERT_TRUE(rig.setup());
  EXPECT_EQ(rig.commands[2], 0.0);
  rig.enable();
  for (const double effort : {2.3, -2.3, 0.0}) {
    rig.commands[2] = effort;
    rig.cycle(0x27);
    EXPECT_EQ(rig.offset(), static_cast<int16_t>(effort * 8));
    EXPECT_DOUBLE_EQ(rig.states[2], static_cast<double>(rig.offset()) / 8);
    EXPECT_EQ(rig.states[3], rig.offset());
  }
}

TEST(CspTorqueOffset, DisableFaultAndIncompleteBusClearPreviousOffset)
{
  for (int fault = 0; fault < 4; ++fault) {
    Rig rig;
    ASSERT_TRUE(rig.setup());
    rig.enable();
    rig.commands[2] = 4;
    rig.cycle(0x27);
    ASSERT_EQ(rig.offset(), 32);
    const double epoch = rig.states[4];
    if (fault == 0) {
      rig.commands[1] = 0;
    }
    if (fault == 1) {
      rig.drive.set_state_is_operational(false);
    }
    rig.cycle(fault == 2 ? 0x08 : 0x27, fault != 3);
    EXPECT_EQ(rig.offset(), 0);
    EXPECT_EQ(rig.states[2], 0.0);
    EXPECT_GT(rig.states[4], epoch);
    if (fault != 0) {
      EXPECT_EQ(rig.commands[2], 0.0);
    }
  }
}

TEST(CspTorqueOffset, RejectsNonCspAndFiniteConversionNeverWraps)
{
  Rig wrong_mode;
  EXPECT_FALSE(wrong_mode.setup("10"));
  Rig rig;
  ASSERT_TRUE(rig.setup());
  rig.enable();
  for (double value :
       {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    rig.commands[2] = value;
    rig.cycle(0x27);
    EXPECT_EQ(rig.offset(), 0);
  }
  rig.commands[2] = 1e10;
  rig.cycle(0x27);
  EXPECT_EQ(rig.offset(), 32767);
  rig.commands[2] = -1e10;
  rig.cycle(0x27);
  EXPECT_EQ(rig.offset(), -32768);
}

TEST(CspTorqueOffset, PluginIsExported)
{
  pluginlib::ClassLoader<ethercat_interface::EcSlave> loader(
    "ethercat_interface", "ethercat_interface::EcSlave");
  EXPECT_NE(loader.createSharedInstance("hw_ethercat/CspTorqueOffsetSlave"), nullptr);
}
}  // namespace
