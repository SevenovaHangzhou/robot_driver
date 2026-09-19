#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <stdexcept>

#include "robot_hw_can/head_state_machine.hpp"

namespace robot_hw_can
{
namespace
{

using namespace std::chrono_literals;

std::array<MotorStatus, 2U> status(
  std::uint8_t first, std::uint8_t second, std::chrono::nanoseconds received_at)
{
  return {{{true, first, received_at}, {true, second, received_at}}};
}

TEST(HeadStateMachineTest, HoldsUntilBothMotorsConfirmTheCurrentEnableAttempt)
{
  HeadStateMachine machine{20ms, 100ms};

  auto actions = machine.step(10ms, false, 0U, status(0U, 0U, 10ms));
  EXPECT_EQ(machine.phase(), HeadPhase::disabled);
  EXPECT_FALSE(actions.allow_motion);

  actions = machine.step(20ms, true, 0U, status(0U, 0U, 20ms));
  EXPECT_EQ(machine.phase(), HeadPhase::enabling);
  EXPECT_TRUE(actions.seed_hold);
  EXPECT_TRUE(actions.send_enable);
  EXPECT_TRUE(actions.send_hold);
  EXPECT_FALSE(actions.allow_motion);

  actions = machine.step(30ms, true, 0U, status(1U, 0U, 30ms));
  EXPECT_EQ(machine.phase(), HeadPhase::enabling);
  EXPECT_TRUE(actions.send_hold);
  EXPECT_FALSE(actions.allow_motion);

  actions = machine.step(40ms, true, 0U, status(1U, 1U, 40ms));
  EXPECT_EQ(machine.phase(), HeadPhase::enabled);
  EXPECT_TRUE(actions.allow_motion);
  EXPECT_FALSE(actions.send_enable);
}

TEST(HeadStateMachineTest, DisablesBothAndLatchesFaultUntilExplicitResetCompletes)
{
  HeadStateMachine machine{20ms, 100ms};
  static_cast<void>(machine.step(10ms, true, 0U, status(0U, 0U, 10ms)));
  static_cast<void>(machine.step(20ms, true, 0U, status(1U, 1U, 20ms)));

  auto actions = machine.step(30ms, true, 0U, status(8U, 1U, 30ms));
  EXPECT_EQ(machine.phase(), HeadPhase::fault_latched);
  EXPECT_TRUE(machine.fault_latched());
  EXPECT_TRUE(actions.send_disable);
  EXPECT_FALSE(actions.allow_motion);

  actions = machine.step(40ms, false, 0U, status(0U, 0U, 40ms));
  EXPECT_EQ(machine.phase(), HeadPhase::fault_latched);
  EXPECT_TRUE(machine.fault_latched());

  actions = machine.step(50ms, false, 1U, status(0U, 0U, 50ms));
  EXPECT_EQ(machine.phase(), HeadPhase::reset_disabling);
  EXPECT_TRUE(actions.send_disable);
  EXPECT_FALSE(actions.send_clear_fault);

  actions = machine.step(60ms, false, 1U, status(8U, 0U, 60ms));
  EXPECT_EQ(machine.phase(), HeadPhase::resetting);
  EXPECT_TRUE(actions.send_clear_fault);

  actions = machine.step(70ms, false, 1U, status(0U, 0U, 70ms));
  EXPECT_EQ(machine.phase(), HeadPhase::disabled);
  EXPECT_FALSE(machine.fault_latched());
  EXPECT_FALSE(actions.allow_motion);
}

TEST(HeadStateMachineTest, EnableTimeoutOrStaleFeedbackFailsClosed)
{
  HeadStateMachine machine{20ms, 50ms};
  static_cast<void>(machine.step(10ms, true, 0U, status(0U, 0U, 10ms)));
  auto actions = machine.step(61ms, true, 0U, status(1U, 0U, 61ms));
  EXPECT_EQ(machine.phase(), HeadPhase::fault_latched);
  EXPECT_TRUE(actions.send_disable);

  HeadStateMachine enabled{20ms, 50ms};
  static_cast<void>(enabled.step(10ms, true, 0U, status(0U, 0U, 10ms)));
  static_cast<void>(enabled.step(20ms, true, 0U, status(1U, 1U, 20ms)));
  actions = enabled.step(50ms, true, 0U, status(1U, 1U, 20ms));
  EXPECT_EQ(enabled.phase(), HeadPhase::fault_latched);
  EXPECT_TRUE(actions.send_disable);
}

TEST(HeadStateMachineTest, DisableRequiresFreshConfirmationFromBothMotors)
{
  HeadStateMachine machine{20ms, 100ms};
  static_cast<void>(machine.step(10ms, true, 0U, status(0U, 0U, 10ms)));
  static_cast<void>(machine.step(20ms, true, 0U, status(1U, 1U, 20ms)));

  auto actions = machine.step(30ms, false, 0U, status(1U, 1U, 30ms));
  EXPECT_EQ(machine.phase(), HeadPhase::disabling);
  EXPECT_TRUE(actions.send_disable);
  EXPECT_FALSE(actions.allow_motion);

  actions = machine.step(40ms, false, 0U, status(0U, 1U, 40ms));
  EXPECT_EQ(machine.phase(), HeadPhase::disabling);
  EXPECT_TRUE(actions.send_disable);

  actions = machine.step(50ms, false, 0U, status(0U, 0U, 50ms));
  EXPECT_EQ(machine.phase(), HeadPhase::disabled);
  EXPECT_FALSE(actions.allow_motion);
}

TEST(CommandRateLimiterTest, EmitsAtConfiguredPeriodAndResetsForImmediateSafetyTraffic)
{
  CommandRateLimiter limiter{4ms};
  EXPECT_TRUE(limiter.due(10ms));
  EXPECT_FALSE(limiter.due(13ms));
  EXPECT_TRUE(limiter.due(14ms));
  limiter.reset();
  EXPECT_TRUE(limiter.due(14ms));
  EXPECT_THROW(static_cast<void>(CommandRateLimiter{0ms}), std::invalid_argument);
}

}  // namespace
}  // namespace robot_hw_can
