#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "robot_hw_ethercat/pp_gripper.hpp"

using robot_hw_ethercat::PpConfig;
using robot_hw_ethercat::PpFeedback;
using robot_hw_ethercat::PpGripper;
using robot_hw_ethercat::PpRequest;
using robot_hw_ethercat::PpState;

namespace
{
// Synthetic calibration and timing, never a production drive configuration.
PpConfig config()
{
  return {100000.0, 100.0, 0.0, 0.08, 10.0, 20.0, 200,
    0.0001, 0.0001, 0.1, 0.2, 2};
}

class PpTest : public testing::Test
{
protected:
  PpGripper drive{config()};
  PpRequest request{0, 0.04, 5.0, true};
  PpFeedback feedback{true, 0x000f, 0x0027, 1, 100, 0.0};
  double time{0.0};

  auto tick(bool sent = true)
  {
    if (sent) {drive.sent();}
    time += 0.004;
    return drive.update(request, feedback, time);
  }

  void start()
  {
    tick();
    request = {1, 0.04, 5.0, false};
    const auto prepared = tick();
    EXPECT_EQ(prepared.target, 4100);
    EXPECT_EQ(prepared.max_torque, 100);
    EXPECT_EQ(prepared.control_word & 0x0110, 0x0100);
    EXPECT_EQ(tick().control_word & 0x0110, 0x0010);
    feedback.status_word |= 0x1000;
    EXPECT_EQ(tick().control_word & 0x0010, 0);
    feedback.status_word &= ~0x1000;
    EXPECT_EQ(tick().state, PpState::moving);
  }
};
}  // namespace

TEST(PpConfiguration, RejectsMissingAndInvalidCalibration)
{
  auto cfg = config();
  cfg.counts_per_metre = 0.0;
  EXPECT_THROW(PpGripper{cfg}, std::invalid_argument);
  cfg = config();
  cfg.permille_per_newton = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PpGripper{cfg}, std::invalid_argument);
  cfg = config();
  cfg.max_force = 11.0;
  EXPECT_THROW(PpGripper{cfg}, std::invalid_argument);
}

TEST_F(PpTest, IdleNeverTriggersAndPreloadsExactRawPosition)
{
  feedback.actual_position = -123456;
  const auto output = tick();
  EXPECT_EQ(output.target, -123456);
  EXPECT_EQ(output.control_word, 0x010f);
  EXPECT_EQ(output.state, PpState::idle);
}

TEST_F(PpTest, LimitAndTargetAreSentBeforeTheNewSetpointEdge)
{
  tick();
  request = {1, 0.04, 5.0, false};
  tick();
  EXPECT_EQ(tick(false).control_word & 0x0010, 0);
  EXPECT_EQ(tick().control_word & 0x0010, 0x0010);
}

TEST_F(PpTest, RequiresAcknowledgmentAndNeverPulsesRepeatedTargets)
{
  start();
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(tick().control_word & 0x0010, 0);
  }
  feedback.actual_position = 4100;
  EXPECT_EQ(tick().state, PpState::moving);
  feedback.status_word |= 0x0400;
  EXPECT_EQ(tick().state, PpState::reached);
  request.sequence = 2;
  EXPECT_EQ(tick().state, PpState::preparing);
  EXPECT_EQ(tick().control_word & 0x0010, 0x0010);
}

TEST_F(PpTest, StaleTargetReachedDoesNotCompleteBeforeHandshake)
{
  feedback.actual_position = 4100;
  feedback.status_word |= 0x0400;
  tick();
  request = {1, 0.04, 5.0, false};
  EXPECT_EQ(tick().state, PpState::preparing);
  EXPECT_EQ(tick().state, PpState::waiting_ack);
}

TEST_F(PpTest, CancellationWaitsForStoppedFeedbackAndRetainsForceLimit)
{
  start();
  request.halt = true;
  feedback.velocity = 0.01;
  auto output = tick();
  EXPECT_EQ(output.state, PpState::halting);
  EXPECT_EQ(output.control_word & 0x0110, 0x0100);
  EXPECT_EQ(output.max_torque, 100);
  EXPECT_EQ(tick().state, PpState::halting);
  feedback.velocity = 0.0;
  EXPECT_EQ(tick().state, PpState::halting);
  EXPECT_EQ(tick().state, PpState::halted);
}

TEST_F(PpTest, InvalidRequestsCannotRaiseTheForceLimitOrMove)
{
  tick();
  for (const double force : {0.0, -1.0, 10.1, std::numeric_limits<double>::infinity()}) {
    request = {request.sequence + 1, 0.04, force, false};
    const auto output = tick();
    EXPECT_EQ(output.state, PpState::fault);
    EXPECT_EQ(output.control_word & 0x0010, 0);
  }
}

TEST_F(PpTest, DisableAndFaultResetAlwaysTakePriorityOverPpBits)
{
  start();
  for (const auto cw : {0x0000, 0x0006, 0x0007, 0x000b, 0x0080}) {
    feedback.control_word = static_cast<uint16_t>(cw);
    EXPECT_EQ(tick().control_word, cw);
  }
  feedback.control_word = 0x000f;
  EXPECT_EQ(tick().state, PpState::idle);
  EXPECT_EQ(tick().control_word & 0x0010, 0);
}

TEST_F(PpTest, FaultAndBusLossInvalidateThePreviousCommand)
{
  start();
  feedback.operational = false;
  EXPECT_EQ(tick().state, PpState::unavailable);
  feedback.operational = true;
  EXPECT_EQ(tick().state, PpState::idle);
  feedback.status_word = 0x0008;
  EXPECT_EQ(tick().state, PpState::fault);
}

TEST_F(PpTest, AckTimeoutLatchesHaltUntilDisabled)
{
  tick();
  request = {1, 0.04, 5.0, false};
  tick();
  tick();
  time += 0.2;
  EXPECT_EQ(tick().state, PpState::fault);
  feedback.status_word |= 0x1000;
  EXPECT_EQ(tick().state, PpState::fault);
  EXPECT_EQ(tick().control_word & 0x0110, 0x0100);
}

TEST_F(PpTest, ModeMismatchAndNonfiniteFeedbackFailClosed)
{
  start();
  feedback.mode = 8;
  EXPECT_EQ(tick().state, PpState::fault);
  feedback.mode = 1;
  feedback.velocity = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(tick().state, PpState::fault);
}

TEST_F(PpTest, EnablingTransitionDoesNotLatchFaultOrReplayCommands)
{
  feedback.status_word = 0x0023;
  EXPECT_EQ(tick().state, PpState::unavailable);
  feedback.status_word = 0x0027;
  EXPECT_EQ(tick().state, PpState::idle);
}

TEST_F(PpTest, CancelBeforeFirstTriggerNeverMoves)
{
  tick();
  request = {1, 0.04, 5.0, true};
  EXPECT_EQ(tick().state, PpState::halting);
  EXPECT_EQ(tick().control_word & 0x0010, 0);
  EXPECT_EQ(tick().state, PpState::halted);
}
