#include <gtest/gtest.h>

#include "bms_node/chinese_status.hpp"

#include <string>

namespace
{

bms_node::PackState healthy_state()
{
  bms_node::PackState state;
  state.sample.voltage_v = 50.0;
  state.sample.soc_fraction = 0.8;
  state.sample.current_a = -12.5;
  state.sample.last_frame_s = 10.0;
  state.temperature_frame_s = 10.0;
  state.max_cell_temperature_c = 25.0;
  state.min_cell_temperature_c = 20.0;
  state.mos_temperature_c = 30.0;
  state.board_temperature_c = 28.0;
  state.temperature_delta_c = 5.0;
  state.mos_frame_s = 10.0;
  state.operating_state_raw = 2U;
  state.charge_mos_raw = 0U;
  state.discharge_mos_raw = 1U;
  state.fault_frame_s = 10.0;
  return state;
}

TEST(ChineseStatusTest, FormatsMeasurementsAndMosStatesInChinese)
{
  const auto state = healthy_state();
  const auto text =
    bms_node::format_chinese_status(state, 1U, 10.1, 1.0, bms_node::ChineseStatusOptions{1, 0});
  EXPECT_NE(text.find("通信状态=正常"), std::string::npos);
  EXPECT_NE(text.find("电量=80.0%"), std::string::npos);
  EXPECT_NE(text.find("电压=50.0V"), std::string::npos);
  EXPECT_NE(text.find("电流=-12.5A"), std::string::npos);
  EXPECT_NE(text.find("充放电状态=放电"), std::string::npos);
  EXPECT_NE(text.find("充电MOS状态=断开"), std::string::npos);
  EXPECT_NE(text.find("放电MOS状态=闭合"), std::string::npos);
}

TEST(ChineseStatusTest, DoesNotGuessUnconfiguredMosMapping)
{
  const auto state = healthy_state();
  const auto text =
    bms_node::format_chinese_status(state, 1U, 10.1, 1.0, bms_node::ChineseStatusOptions{});
  EXPECT_NE(text.find("充电MOS状态=未知（原始值0）"), std::string::npos);
  EXPECT_NE(text.find("放电MOS状态=未知（原始值1）"), std::string::npos);
}

TEST(ChineseStatusTest, ExpandsFaultBitsAndFaultCodeInChinese)
{
  auto state = healthy_state();
  state.fault_bytes[0U] = 0x01U;
  state.fault_bytes[5U] = 0x20U;
  state.fault_bytes[7U] = 3U;
  const auto text = bms_node::format_chinese_fault(state, 2U, 10.1, 1.0);
  EXPECT_NE(text.find("单体电压过高一级告警"), std::string::npos);
  EXPECT_NE(text.find("预充失败"), std::string::npos);
  EXPECT_NE(text.find("故障码3"), std::string::npos);
}

TEST(ChineseStatusTest, ReportsNoFaultAndTimeoutInChinese)
{
  auto state = healthy_state();
  EXPECT_NE(bms_node::format_chinese_fault(state, 1U, 10.1, 1.0).find("无故障"), std::string::npos);
  EXPECT_NE(
    bms_node::format_chinese_fault(state, 1U, 12.0, 1.0).find("故障状态通信超时"),
    std::string::npos);
}

}  // namespace
