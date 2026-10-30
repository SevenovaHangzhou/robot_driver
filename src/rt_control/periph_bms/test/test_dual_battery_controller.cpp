#include <gtest/gtest.h>

#include "periph_bms/dual_battery_controller.hpp"

namespace
{

periph_bms::PackState healthy_pack(const double now, const std::uint8_t mos)
{
  periph_bms::PackState pack;
  pack.sample.voltage_v = 50.0;
  pack.sample.current_a = 0.0;
  pack.sample.soc_fraction = 0.8;
  pack.sample.last_frame_s = now;
  pack.fault_frame_s = now;
  pack.mos_frame_s = now;
  pack.operating_state_raw = 0U;
  pack.discharge_mos_raw = mos;
  pack.pack_info_s = now;
  pack.charger_connected_raw = 0U;
  pack.cell_count = 3U;
  pack.cell_voltage_v[0] = 3.3;
  pack.cell_voltage_v[1] = 3.3;
  pack.cell_voltage_v[2] = 3.3;
  pack.cell_frame_s[0] = now;
  pack.cell_extremes_s = now;
  pack.min_cell_voltage_v = 3.3;
  pack.max_cell_voltage_v = 3.3;
  pack.temperature_frame_s = now;
  pack.min_cell_temperature_c = 20.0;
  pack.max_cell_temperature_c = 25.0;
  return pack;
}

periph_bms::DischargeSettings settings()
{
  return {10.0, 0.5, 0.4, 0.6, 0.5, 1.0, 5.0, 3.0, 3.7, 0.0, 45.0, 1U, 0U};
}

periph_bms::DischargeInputs inputs_for(
  const periph_bms::PackState & primary, const periph_bms::PackState & secondary, const double now)
{
  return {primary, secondary, now, true, now, false, now, true};
}

void reach_join_check(
  periph_bms::DualBatteryController & controller, periph_bms::DischargeInputs & inputs)
{
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kWaitingForJoinConditions);
}

void reach_running(
  periph_bms::DualBatteryController & controller, periph_bms::PackState & secondary,
  periph_bms::DischargeInputs & inputs)
{
  reach_join_check(controller, inputs);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  inputs.now_s += 0.7;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kEnableSecondary);
  secondary.discharge_ack_s = inputs.now_s + 0.1;
  secondary.discharge_ack_value = 1U;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  secondary.discharge_mos_raw = 1U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kCloseRelay);
  inputs.relay_closed = true;
  inputs.relay_feedback_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  inputs.now_s += 0.5;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kRunning);
}

TEST(DualBatteryControllerTest, JoinsOnlyAfterStableVoltageAckMosAndRelayFeedback)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
}

TEST(DualBatteryControllerTest, MissingRelayOrLoadFeedbackNeverClosesRelay)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  inputs.relay_feedback_s.reset();
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  inputs.relay_feedback_s = 1.0;
  reach_join_check(controller, inputs);
  inputs.loads_stopped_s.reset();
  inputs.now_s = 1.7;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
}

TEST(DualBatteryControllerTest, ExcessJoinDeltaResetsStabilityWindow)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_join_check(controller, inputs);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  secondary.sample.voltage_v = 51.0;
  inputs.now_s = 1.4;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  secondary.sample.voltage_v = 50.0;
  inputs.now_s = 1.6;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  inputs.now_s = 1.9;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
}

TEST(DualBatteryControllerTest, SecondaryFaultDisablesOnlySecondaryAndOpensRelay)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  secondary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisableSecondary);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisableSecondary);
  secondary.discharge_mos_raw = 0U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kOpenRelay);
  inputs.relay_closed = false;
  inputs.relay_feedback_s = inputs.now_s;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kFaultLatched);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kOpenRelay);
}

TEST(DualBatteryControllerTest, VoltageDriftBeforeRelayClosesStopsJoin)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_join_check(controller, inputs);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  inputs.now_s = 1.7;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kEnableSecondary);
  secondary.sample.voltage_v = 51.0;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisableSecondary);
  EXPECT_TRUE(controller.secondary_faulted());
  EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kStoppingSecondary);
}

TEST(DualBatteryControllerTest, PrimaryFaultLeavesHealthyConnectedSecondary)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  primary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisablePrimary);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisablePrimary);
  primary.discharge_mos_raw = 0U;
  primary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kFaultLatched);
  EXPECT_FALSE(controller.secondary_faulted());
  EXPECT_EQ(inputs.relay_closed, true);
}

TEST(DualBatteryControllerTest, PrimaryFaultBeforeSecondaryJoinDisablesPrimaryOnly)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_join_check(controller, inputs);
  primary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisablePrimary);
  EXPECT_FALSE(controller.secondary_faulted());
  EXPECT_EQ(inputs.relay_closed, false);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisablePrimary);
}

TEST(DualBatteryControllerTest, PrimaryFaultAfterSecondaryIsolationStillDisablesPrimary)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  secondary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisableSecondary);
  secondary.discharge_mos_raw = 0U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kOpenRelay);
  inputs.relay_closed = false;
  inputs.relay_feedback_s = inputs.now_s;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  primary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisablePrimary);
}

TEST(DualBatteryControllerTest, SecondaryFaultAfterPrimaryIsolationStillOpensK2)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  primary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisablePrimary);
  primary.discharge_mos_raw = 0U;
  primary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  secondary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisableSecondary);
  secondary.discharge_mos_raw = 0U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kOpenRelay);
}

TEST(DualBatteryControllerTest, VoltageTripIsolatesSecondary)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  secondary.sample.voltage_v = 51.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisableSecondary);
}

TEST(DualBatteryControllerTest, MissingMosOffReplyRequiresLowCurrentBeforeRelayOpens)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  secondary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisableSecondary);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisableSecondary);
  inputs.now_s += 0.6;
  secondary.sample.current_a = 8.0;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  secondary.sample.current_a = 0.0;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kOpenRelay);
}

TEST(DualBatteryControllerTest, BothFaultedPacksReceiveMosOffCommands)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  primary.fault_bytes[0] = 1U;
  secondary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisableSecondary);
  secondary.discharge_mos_raw = 0U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kOpenRelay);
  inputs.relay_closed = false;
  inputs.relay_feedback_s = inputs.now_s;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisablePrimary);
  primary.discharge_mos_raw = 0U;
  primary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kFaultLatched);
}

TEST(DualBatteryControllerTest, UnconfirmedLoadStopPreventsMosSwitch)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  secondary.fault_bytes[0] = 1U;
  inputs.loads_stopped = false;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  inputs.loads_stopped = true;
  secondary.sample.current_a = 8.0;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
}

TEST(DualBatteryControllerTest, MissingStatusAtStartupDoesNotEnergizeRelay)
{
  periph_bms::DualBatteryController controller{settings()};
  periph_bms::PackState primary;
  periph_bms::PackState secondary;
  auto inputs = inputs_for(primary, secondary, 1.0);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kWaitingForPrimary);
}

TEST(DualBatteryControllerTest, ClosedRelayAtStartupStopsLoadBeforeIsolation)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 1U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  inputs.relay_closed = true;
  inputs.loads_stopped = false;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kStoppingLoads);
  inputs.loads_stopped = true;
  secondary.sample.current_a = 8.0;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  secondary.sample.current_a = 0.0;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisableSecondary);
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kDisableSecondary);
  secondary.discharge_mos_raw = 0U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kOpenRelay);
}

TEST(DualBatteryControllerTest, RejectsInvalidTripThreshold)
{
  auto invalid = settings();
  invalid.trip_voltage_delta_v = invalid.max_voltage_delta_v - 0.1;
  EXPECT_THROW(periph_bms::DualBatteryController controller{invalid}, std::invalid_argument);
}

TEST(DualBatteryControllerReviewTest, D9RetriesRequireFreshStoppedLowCurrentFeedback)
{
  for (const bool primary_fault : {false, true}) {
    periph_bms::DualBatteryController controller{settings()};
    auto primary = healthy_pack(1.0, 1U);
    auto secondary = healthy_pack(1.0, 0U);
    auto inputs = inputs_for(primary, secondary, 1.0);
    reach_running(controller, secondary, inputs);
    (primary_fault ? primary : secondary).fault_bytes[0] = 1U;
    const auto command = primary_fault ? periph_bms::DischargeAction::kDisablePrimary
                                       : periph_bms::DischargeAction::kDisableSecondary;
    ASSERT_EQ(controller.update(inputs), command);
    inputs.loads_stopped = false;
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
    inputs.loads_stopped = true;
    secondary.sample.current_a = 8.0;
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
    secondary.sample.current_a = 0.0;
    inputs.loads_stopped_s.reset();
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
    inputs.loads_stopped_s = inputs.now_s;
    secondary.sample.last_frame_s.reset();
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
    secondary.sample.last_frame_s = inputs.now_s;
    EXPECT_EQ(controller.update(inputs), command);
  }
}

TEST(DualBatteryControllerReviewTest, UnexpectedPrimaryMosOffStopsWithoutChoosingIsolation)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  primary.discharge_mos_raw = 0U;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kAwaitingIsolationPolicy);
  // Even after stop confirmation, no automatic K2 opening or MOS isolation policy.
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  primary.discharge_mos_raw = 1U;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
}

TEST(DualBatteryControllerReviewTest, PrimaryMosOffDuringHandshakeNeverClosesRelay)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_join_check(controller, inputs);
  static_cast<void>(controller.update(inputs));
  inputs.now_s = 1.7;
  ASSERT_EQ(controller.update(inputs), periph_bms::DischargeAction::kEnableSecondary);
  primary.discharge_mos_raw = 0U;
  secondary.discharge_ack_s = 1.8;
  secondary.discharge_ack_value = 1U;
  inputs.now_s = 1.8;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  secondary.discharge_mos_raw = 1U;
  secondary.mos_frame_s = 1.9;
  inputs.now_s = 1.9;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
}

TEST(DualBatteryControllerReviewTest, StaleRelayFeedbackDuringHandshakeBlocksClosing)
{
  auto config = settings();
  config.status_timeout_s = 0.25;
  config.stable_s = 0.1;
  periph_bms::DualBatteryController controller{config};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_join_check(controller, inputs);
  static_cast<void>(controller.update(inputs));
  inputs.now_s = 1.15;
  ASSERT_EQ(controller.update(inputs), periph_bms::DischargeAction::kEnableSecondary);
  secondary.discharge_ack_s = 1.2;
  secondary.discharge_ack_value = 1U;
  inputs.now_s = 1.2;
  static_cast<void>(controller.update(inputs));
  primary = healthy_pack(1.3, 1U);
  secondary = healthy_pack(1.3, 1U);
  inputs.loads_stopped_s = 1.3;
  inputs.now_s = 1.3;
  EXPECT_NE(controller.update(inputs), periph_bms::DischargeAction::kCloseRelay);
  EXPECT_TRUE(controller.secondary_faulted());
}

TEST(DualBatteryControllerReviewTest, UnexpectedRelayClosureBlocksD9AndJoin)
{
  for (const bool already_sent : {false, true}) {
    periph_bms::DualBatteryController controller{settings()};
    auto primary = healthy_pack(1.0, 1U);
    auto secondary = healthy_pack(1.0, 0U);
    auto inputs = inputs_for(primary, secondary, 1.0);
    reach_join_check(controller, inputs);
    static_cast<void>(controller.update(inputs));
    inputs.now_s = 1.7;
    if (already_sent) {
      ASSERT_EQ(controller.update(inputs), periph_bms::DischargeAction::kEnableSecondary);
    }
    inputs.relay_closed = true;
    inputs.loads_stopped = false;
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
    EXPECT_TRUE(controller.secondary_faulted());
  }
}

TEST(DualBatteryControllerReviewTest, UndefinedOperatingStatesNeverEnableSecondary)
{
  for (const std::uint8_t raw : std::array<std::uint8_t, 3U>{3U, 127U, 255U}) {
    periph_bms::DualBatteryController controller{settings()};
    auto primary = healthy_pack(1.0, 1U);
    auto secondary = healthy_pack(1.0, 0U);
    primary.operating_state_raw = raw;
    auto inputs = inputs_for(primary, secondary, 1.0);
    for (int tick = 0; tick < 5; ++tick) {
      EXPECT_NE(controller.update(inputs), periph_bms::DischargeAction::kEnableSecondary);
      inputs.now_s += 0.2;
    }
  }
}

TEST(DualBatteryControllerReviewTest, MissingSecondaryRecoversFromIdleFeedbackGap)
{
  for (const bool mos_gap : {false, true}) {
    auto config = settings();
    config.status_timeout_s = 0.5;
    periph_bms::DualBatteryController controller{config};
    auto primary = healthy_pack(1.0, 1U);
    periph_bms::PackState secondary;
    auto inputs = inputs_for(primary, secondary, 1.0);
    static_cast<void>(controller.update(inputs));
    static_cast<void>(controller.update(inputs));
    ASSERT_EQ(controller.phase(), periph_bms::DischargePhase::kWaitingForSecondary);
    primary = healthy_pack(2.0, 1U);
    inputs.now_s = 2.0;
    inputs.loads_stopped_s = 2.0;
    if (mos_gap) {
      primary.mos_frame_s = 1.0;
      inputs.relay_feedback_s = 2.0;
    }
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
    EXPECT_FALSE(controller.secondary_faulted());
    for (int tick = 0; tick < 20; ++tick) {
      inputs.now_s += 0.2;
      primary = healthy_pack(inputs.now_s, 1U);
      inputs.loads_stopped_s = inputs.now_s;
      inputs.relay_feedback_s = inputs.now_s;
      EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
      EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kWaitingForSecondary);
    }
  }
}

TEST(DualBatteryControllerReviewTest, JoinWaitFeedbackGapResetsStabilityAndRecovers)
{
  for (const bool mos_gap : {false, true}) {
    auto config = settings();
    config.status_timeout_s = 0.5;
    periph_bms::DualBatteryController controller{config};
    auto primary = healthy_pack(1.0, 1U);
    auto secondary = healthy_pack(1.0, 0U);
    secondary.sample.voltage_v = 52.0;
    auto inputs = inputs_for(primary, secondary, 1.0);
    reach_join_check(controller, inputs);
    static_cast<void>(controller.update(inputs));
    auto refresh = [&](const double now) {
      primary = healthy_pack(now, 1U);
      secondary = healthy_pack(now, 0U);
      inputs.now_s = now;
      inputs.loads_stopped_s = now;
      inputs.relay_feedback_s = now;
    };
    refresh(2.0);
    secondary.sample.voltage_v = 52.0;
    if (mos_gap) {
      primary.mos_frame_s = 1.0;
    } else {
      inputs.relay_feedback_s = 1.0;
    }
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
    EXPECT_FALSE(controller.secondary_faulted());
    EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kWaitingForJoinConditions);
    refresh(2.1);
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
    refresh(2.5);
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
    if (mos_gap) {
      primary.mos_frame_s = 1.0;
    } else {
      inputs.relay_feedback_s = 1.0;
    }
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
    refresh(2.8);
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
    refresh(3.2);
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kNone);
    refresh(3.5);
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kEnableSecondary);
  }
}

TEST(DualBatteryControllerReviewTest, FreshUnexpectedClosureStillFaultsWithAbsentSecondary)
{
  periph_bms::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  periph_bms::PackState secondary;
  auto inputs = inputs_for(primary, secondary, 1.0);
  static_cast<void>(controller.update(inputs));
  static_cast<void>(controller.update(inputs));
  ASSERT_EQ(controller.phase(), periph_bms::DischargePhase::kWaitingForSecondary);
  inputs.relay_closed = true;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  EXPECT_TRUE(controller.secondary_faulted());
}

TEST(DualBatteryControllerReviewTest, PrimaryMosGapAfterD9StillLatchesFault)
{
  auto config = settings();
  config.status_timeout_s = 0.5;
  config.stable_s = 0.1;
  periph_bms::DualBatteryController controller{config};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_join_check(controller, inputs);
  static_cast<void>(controller.update(inputs));
  inputs.now_s = 1.2;
  ASSERT_EQ(controller.update(inputs), periph_bms::DischargeAction::kEnableSecondary);
  primary = healthy_pack(1.6, 1U);
  secondary = healthy_pack(1.6, 0U);
  primary.mos_frame_s = 1.0;
  inputs.now_s = 1.6;
  inputs.loads_stopped_s = 1.6;
  inputs.relay_feedback_s = 1.6;
  inputs.loads_stopped = false;
  EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
  EXPECT_TRUE(controller.secondary_faulted());
  EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kStoppingLoads);
}

TEST(DualBatteryControllerReviewTest, WaitingStillLatchesNonMosStatusLossAndRealFaults)
{
  for (const bool fault_bit : {false, true}) {
    auto config = settings();
    config.status_timeout_s = 0.5;
    periph_bms::DualBatteryController controller{config};
    auto primary = healthy_pack(1.0, 1U);
    periph_bms::PackState secondary;
    auto inputs = inputs_for(primary, secondary, 1.0);
    static_cast<void>(controller.update(inputs));
    static_cast<void>(controller.update(inputs));
    primary = healthy_pack(2.0, 1U);
    if (fault_bit) {
      primary.fault_bytes[0] = 1U;
    } else {
      primary.sample.last_frame_s = 1.0;
    }
    inputs.now_s = 2.0;
    inputs.relay_feedback_s = 2.0;
    inputs.loads_stopped_s = 2.0;
    EXPECT_EQ(controller.update(inputs), periph_bms::DischargeAction::kStopLoads);
    EXPECT_EQ(controller.phase(), periph_bms::DischargePhase::kStoppingLoads);
  }
}

}  // namespace
