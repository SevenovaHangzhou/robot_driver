#include <gtest/gtest.h>

#include "bms_node/dual_battery_controller.hpp"

namespace
{

bms_node::PackState healthy_pack(const double now, const std::uint8_t mos)
{
  bms_node::PackState pack;
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

bms_node::DischargeSettings settings()
{
  return {10.0, 0.5, 0.4, 0.6, 0.5, 1.0, 5.0, 3.0, 3.7, 0.0, 45.0, 1U, 0U};
}

bms_node::DischargeInputs inputs_for(
  const bms_node::PackState & primary, const bms_node::PackState & secondary,
  const double now)
{
  return {primary, secondary, now, true, now, false, now, true};
}

void reach_join_check(
  bms_node::DualBatteryController & controller, bms_node::DischargeInputs & inputs)
{
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), bms_node::DischargePhase::kWaitingForJoinConditions);
}

void reach_running(
  bms_node::DualBatteryController & controller,
  bms_node::PackState & secondary, bms_node::DischargeInputs & inputs)
{
  reach_join_check(controller, inputs);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  inputs.now_s += 0.7;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kEnableSecondary);
  secondary.discharge_ack_s = inputs.now_s + 0.1;
  secondary.discharge_ack_value = 1U;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  secondary.discharge_mos_raw = 1U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kCloseRelay);
  inputs.relay_closed = true;
  inputs.relay_feedback_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  inputs.now_s += 0.5;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), bms_node::DischargePhase::kRunning);
}

TEST(DualBatteryControllerTest, JoinsOnlyAfterStableVoltageAckMosAndRelayFeedback)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
}

TEST(DualBatteryControllerTest, MissingRelayOrLoadFeedbackNeverClosesRelay)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  inputs.relay_feedback_s.reset();
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kStopLoads);
  inputs.relay_feedback_s = 1.0;
  reach_join_check(controller, inputs);
  inputs.loads_stopped_s.reset();
  inputs.now_s = 1.7;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
}

TEST(DualBatteryControllerTest, ExcessJoinDeltaResetsStabilityWindow)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_join_check(controller, inputs);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  secondary.sample.voltage_v = 51.0;
  inputs.now_s = 1.4;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  secondary.sample.voltage_v = 50.0;
  inputs.now_s = 1.6;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  inputs.now_s = 1.9;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
}

TEST(DualBatteryControllerTest, SecondaryFaultDisablesOnlySecondaryAndOpensRelay)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  secondary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisableSecondary);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisableSecondary);
  secondary.discharge_mos_raw = 0U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kOpenRelay);
  inputs.relay_closed = false;
  inputs.relay_feedback_s = inputs.now_s;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), bms_node::DischargePhase::kFaultLatched);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kOpenRelay);
}

TEST(DualBatteryControllerTest, VoltageDriftBeforeRelayClosesStopsJoin)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_join_check(controller, inputs);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  inputs.now_s = 1.7;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kEnableSecondary);
  secondary.sample.voltage_v = 51.0;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisableSecondary);
  EXPECT_TRUE(controller.secondary_faulted());
  EXPECT_EQ(controller.phase(), bms_node::DischargePhase::kStoppingSecondary);
}

TEST(DualBatteryControllerTest, PrimaryFaultLeavesHealthyConnectedSecondary)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  primary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisablePrimary);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisablePrimary);
  primary.discharge_mos_raw = 0U;
  primary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), bms_node::DischargePhase::kFaultLatched);
  EXPECT_FALSE(controller.secondary_faulted());
  EXPECT_EQ(inputs.relay_closed, true);
}

TEST(DualBatteryControllerTest, PrimaryFaultBeforeSecondaryJoinDisablesPrimaryOnly)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_join_check(controller, inputs);
  primary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisablePrimary);
  EXPECT_FALSE(controller.secondary_faulted());
  EXPECT_EQ(inputs.relay_closed, false);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisablePrimary);
}

TEST(DualBatteryControllerTest, PrimaryFaultAfterSecondaryIsolationStillDisablesPrimary)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  secondary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisableSecondary);
  secondary.discharge_mos_raw = 0U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kOpenRelay);
  inputs.relay_closed = false;
  inputs.relay_feedback_s = inputs.now_s;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  primary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kStopLoads);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisablePrimary);
}

TEST(DualBatteryControllerTest, SecondaryFaultAfterPrimaryIsolationStillOpensK2)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  primary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisablePrimary);
  primary.discharge_mos_raw = 0U;
  primary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  secondary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kStopLoads);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisableSecondary);
  secondary.discharge_mos_raw = 0U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kOpenRelay);
}

TEST(DualBatteryControllerTest, VoltageTripIsolatesSecondary)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  secondary.sample.voltage_v = 51.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisableSecondary);
}

TEST(DualBatteryControllerTest, MissingMosOffReplyRequiresLowCurrentBeforeRelayOpens)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  secondary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisableSecondary);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisableSecondary);
  inputs.now_s += 0.6;
  secondary.sample.current_a = 8.0;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kStopLoads);
  secondary.sample.current_a = 0.0;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kOpenRelay);
}

TEST(DualBatteryControllerTest, BothFaultedPacksReceiveMosOffCommands)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  primary.fault_bytes[0] = 1U;
  secondary.fault_bytes[0] = 1U;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisableSecondary);
  secondary.discharge_mos_raw = 0U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kOpenRelay);
  inputs.relay_closed = false;
  inputs.relay_feedback_s = inputs.now_s;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisablePrimary);
  primary.discharge_mos_raw = 0U;
  primary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), bms_node::DischargePhase::kFaultLatched);
}

TEST(DualBatteryControllerTest, UnconfirmedLoadStopPreventsMosSwitch)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 0U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  reach_running(controller, secondary, inputs);
  secondary.fault_bytes[0] = 1U;
  inputs.loads_stopped = false;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kStopLoads);
  inputs.loads_stopped = true;
  secondary.sample.current_a = 8.0;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kStopLoads);
}

TEST(DualBatteryControllerTest, MissingStatusAtStartupDoesNotEnergizeRelay)
{
  bms_node::DualBatteryController controller{settings()};
  bms_node::PackState primary;
  bms_node::PackState secondary;
  auto inputs = inputs_for(primary, secondary, 1.0);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kNone);
  EXPECT_EQ(controller.phase(), bms_node::DischargePhase::kWaitingForPrimary);
}

TEST(DualBatteryControllerTest, ClosedRelayAtStartupStopsLoadBeforeIsolation)
{
  bms_node::DualBatteryController controller{settings()};
  auto primary = healthy_pack(1.0, 1U);
  auto secondary = healthy_pack(1.0, 1U);
  auto inputs = inputs_for(primary, secondary, 1.0);
  inputs.relay_closed = true;
  inputs.loads_stopped = false;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kStopLoads);
  EXPECT_EQ(controller.phase(), bms_node::DischargePhase::kStoppingLoads);
  inputs.loads_stopped = true;
  secondary.sample.current_a = 8.0;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kStopLoads);
  secondary.sample.current_a = 0.0;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisableSecondary);
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kDisableSecondary);
  secondary.discharge_mos_raw = 0U;
  secondary.mos_frame_s = inputs.now_s + 0.1;
  inputs.now_s += 0.1;
  EXPECT_EQ(controller.update(inputs), bms_node::DischargeAction::kOpenRelay);
}

TEST(DualBatteryControllerTest, RejectsInvalidTripThreshold)
{
  auto invalid = settings();
  invalid.trip_voltage_delta_v = invalid.max_voltage_delta_v - 0.1;
  EXPECT_THROW(bms_node::DualBatteryController controller{invalid}, std::invalid_argument);
}

}  // namespace
