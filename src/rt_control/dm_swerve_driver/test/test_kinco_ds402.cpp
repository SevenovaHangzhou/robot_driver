#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "dm_swerve_driver/kinco_ds402.hpp"
#include "dm_swerve_driver/kinco_pdo.hpp"

namespace dm_swerve_driver {
namespace {

TEST(KincoDs402Test, DecodesStandardStatuswordStatesWithMasks)
{
  const std::array<std::pair<std::uint16_t, Ds402State>, 8U> cases{{
      {0x0000U, Ds402State::not_ready_to_switch_on},
      {0x0040U, Ds402State::switch_on_disabled},
      {0x0021U, Ds402State::ready_to_switch_on},
      {0x0023U, Ds402State::switched_on},
      {0x0027U, Ds402State::operation_enabled},
      {0x0007U, Ds402State::quick_stop_active},
      {0x000FU, Ds402State::fault_reaction_active},
      {0x0008U, Ds402State::fault}}};

  for (const auto & [statusword, expected] : cases) {
    EXPECT_EQ(decode_ds402_state(statusword), expected);
    EXPECT_EQ(decode_ds402_state(static_cast<std::uint16_t>(statusword | 0x0200U)), expected);
  }
  EXPECT_EQ(decode_ds402_state(0xFFFFU), Ds402State::unknown);
}

TEST(KincoDs402Test, PlansStrictEnableSequenceAndSafeFallbacks)
{
  EXPECT_EQ(ds402_enable_control_word(Ds402State::not_ready_to_switch_on), 0x0000U);
  EXPECT_EQ(ds402_enable_control_word(Ds402State::switch_on_disabled), 0x0006U);
  EXPECT_EQ(ds402_enable_control_word(Ds402State::ready_to_switch_on), 0x0007U);
  EXPECT_EQ(ds402_enable_control_word(Ds402State::switched_on), 0x000FU);
  EXPECT_EQ(ds402_enable_control_word(Ds402State::operation_enabled), 0x000FU);
  EXPECT_EQ(ds402_enable_control_word(Ds402State::fault), 0x0000U);
  EXPECT_EQ(ds402_enable_control_word(Ds402State::unknown), 0x0000U);
}

TEST(KincoDs402Test, FaultResetRequestProducesOneBoundedBitSevenPulse)
{
  Ds402FaultResetSequence sequence;
  EXPECT_EQ(sequence.next_control_word(Ds402State::fault), 0x0000U);

  sequence.request();
  EXPECT_TRUE(sequence.active());
  EXPECT_EQ(sequence.next_control_word(Ds402State::fault), 0x0000U);
  EXPECT_EQ(sequence.next_control_word(Ds402State::fault), 0x0080U);
  EXPECT_EQ(sequence.next_control_word(Ds402State::fault), 0x0000U);
  EXPECT_FALSE(sequence.active());
  EXPECT_EQ(sequence.next_control_word(Ds402State::fault), 0x0000U);
}

TEST(KincoDs402Test, LeavingFaultCancelsPendingResetAndUsesEnablePlanner)
{
  Ds402FaultResetSequence sequence;
  sequence.request();
  EXPECT_EQ(sequence.next_control_word(Ds402State::fault), 0x0000U);
  EXPECT_EQ(sequence.next_control_word(Ds402State::switch_on_disabled), 0x0006U);
  EXPECT_FALSE(sequence.active());
}

TEST(KincoDs402Test, ClassifiesOnlyLowVoltageAsAutomaticallyRecoverable)
{
  EXPECT_EQ(classify_kinco_error_word(0x0000U).disposition, FaultDisposition::none);

  const auto low_voltage = classify_kinco_error_word((1U << 6U) | (1U << 10U));
  EXPECT_TRUE(low_voltage.under_voltage);
  EXPECT_EQ(low_voltage.disposition, FaultDisposition::recoverable);

  const auto encoder = classify_kinco_error_word(1U << 2U);
  EXPECT_TRUE(encoder.encoder);
  EXPECT_EQ(encoder.disposition, FaultDisposition::latch);

  const auto following = classify_kinco_error_word(1U << 9U);
  EXPECT_TRUE(following.following_error);
  EXPECT_EQ(following.disposition, FaultDisposition::latch);

  const auto mixed = classify_kinco_error_word((1U << 6U) | (1U << 7U));
  EXPECT_TRUE(mixed.over_current);
  EXPECT_EQ(mixed.disposition, FaultDisposition::latch);
}

TEST(KincoDs402Test, DeclaresTypedCyclicPdoContract)
{
  static_assert(std::is_same_v<decltype(KincoAxisCommand{}.target_position), std::int32_t>);
  static_assert(std::is_same_v<decltype(KincoAxisCommand{}.target_velocity), std::int32_t>);
  static_assert(std::is_same_v<decltype(KincoAxisCommand{}.target_torque_percent), std::int16_t>);
  static_assert(std::is_same_v<decltype(KincoAxisFeedback{}.actual_position), std::int32_t>);
  static_assert(std::is_same_v<decltype(KincoAxisFeedback{}.error_word), std::uint16_t>);

  EXPECT_EQ(static_cast<std::int8_t>(KincoOperationMode::cyclic_synchronous_position), 8);
  EXPECT_EQ(static_cast<std::int8_t>(KincoOperationMode::cyclic_synchronous_velocity), 9);
  EXPECT_EQ(static_cast<std::int8_t>(KincoOperationMode::cyclic_synchronous_torque), 10);
  EXPECT_EQ(kKincoControlword.index, 0x6040U);
  EXPECT_EQ(kKincoTargetPosition.index, 0x607AU);
  EXPECT_EQ(kKincoTargetVelocity.index, 0x60FFU);
  EXPECT_EQ(kKincoTargetTorque.index, 0x6071U);
  EXPECT_EQ(kKincoErrorWord.index, 0x2601U);
}

}  // namespace
}  // namespace dm_swerve_driver
