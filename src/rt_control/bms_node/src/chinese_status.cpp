#include "bms_node/chinese_status.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

namespace bms_node
{
namespace
{

bool fresh(const std::optional<double> & stamp, const double now_s, const double timeout_s)
{
  return stamp && std::isfinite(*stamp) && std::isfinite(now_s) && std::isfinite(timeout_s) &&
         timeout_s > 0.0 && now_s >= *stamp && now_s - *stamp <= timeout_s;
}

std::string number_or_unknown(
  const std::optional<double> & value, const bool valid, const int precision,
  const std::string_view unit)
{
  if (!valid || !value || !std::isfinite(*value)) {
    return "未知";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << *value << unit;
  return stream.str();
}

std::string operating_state(const std::optional<std::uint8_t> value, const bool valid)
{
  if (!valid || !value) {
    return "未知";
  }
  switch (*value) {
    case 0U:
      return "静止";
    case 1U:
      return "充电";
    case 2U:
      return "放电";
    default:
      return "未知（原始值" + std::to_string(*value) + "）";
  }
}

std::string mos_state(
  const std::optional<std::uint8_t> value, const bool valid, const ChineseStatusOptions & options)
{
  if (!valid || !value) {
    return "未知";
  }
  if (options.mos_closed_raw >= 0 && *value == options.mos_closed_raw) {
    return "闭合";
  }
  if (options.mos_open_raw >= 0 && *value == options.mos_open_raw) {
    return "断开";
  }
  return "未知（原始值" + std::to_string(*value) + "）";
}

constexpr std::array<std::array<std::string_view, 8U>, 7U> kFaultNames{{
  {{"单体电压过高一级告警", "单体电压过高二级告警", "单体电压过低一级告警", "单体电压过低二级告警",
    "总压过高一级告警", "总压过高二级告警", "总压过低一级告警", "总压过低二级告警"}},
  {{"充电温度过高一级告警", "充电温度过高二级告警", "充电温度过低一级告警", "充电温度过低二级告警",
    "放电温度过高一级告警", "放电温度过高二级告警", "放电温度过低一级告警",
    "放电温度过低二级告警"}},
  {{"充电过流一级告警", "充电过流二级告警", "放电过流一级告警", "放电过流二级告警",
    "电量过高一级告警", "电量过高二级告警", "电量过低一级告警", "电量过低二级告警"}},
  {{"压差过大一级告警", "压差过大二级告警", "温差过大一级告警", "温差过大二级告警", "", "", "",
    ""}},
  {{"充电MOS过温告警", "放电MOS过温告警", "充电MOS温度传感器故障", "放电MOS温度传感器故障",
    "充电MOS粘连故障", "放电MOS粘连故障", "充电MOS断路故障", "放电MOS断路故障"}},
  {{"AFE采集芯片故障", "单体采集掉线", "单体温度传感器故障", "EEPROM存储故障", "RTC时钟故障",
    "预充失败", "整车通信故障", "内网通信模块故障"}},
  {{"电流模块故障", "内总压检测模块故障", "短路保护故障", "低压禁止充电故障", "GPS或软开关断开MOS",
    "充电柜离线", "热失控故障", ""}},
}};

}  // namespace

std::string format_chinese_status(
  const PackState & state, const std::uint8_t address, const double now_s, const double timeout_s,
  const ChineseStatusOptions & options)
{
  const bool total_valid = state.sample.is_fresh(now_s, timeout_s);
  const bool temperature_valid = fresh(state.temperature_frame_s, now_s, timeout_s);
  const bool mos_valid = fresh(state.mos_frame_s, now_s, timeout_s);
  const bool all_valid = total_valid && temperature_valid && mos_valid;
  std::ostringstream stream;
  stream << "电池地址=0x" << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<unsigned int>(address) << std::dec << std::setfill(' ')
         << "；通信状态=" << (all_valid ? "正常" : "数据缺失或超时") << "；电量=";
  if (total_valid && state.sample.soc_fraction && std::isfinite(*state.sample.soc_fraction)) {
    stream << std::fixed << std::setprecision(1) << *state.sample.soc_fraction * 100.0 << "%";
  } else {
    stream << "未知";
  }
  stream << "；电压=" << number_or_unknown(state.sample.voltage_v, total_valid, 1, "V")
         << "；电流=" << number_or_unknown(state.sample.current_a, total_valid, 1, "A")
         << "；最高单体温度="
         << number_or_unknown(state.max_cell_temperature_c, temperature_valid, 1, "℃")
         << "；最低单体温度="
         << number_or_unknown(state.min_cell_temperature_c, temperature_valid, 1, "℃")
         << "；MOS温度=" << number_or_unknown(state.mos_temperature_c, temperature_valid, 1, "℃")
         << "；板卡温度=" << number_or_unknown(state.board_temperature_c, temperature_valid, 1, "℃")
         << "；温差=" << number_or_unknown(state.temperature_delta_c, temperature_valid, 1, "℃")
         << "；充放电状态=" << operating_state(state.operating_state_raw, mos_valid)
         << "；充电MOS状态=" << mos_state(state.charge_mos_raw, mos_valid, options)
         << "；放电MOS状态=" << mos_state(state.discharge_mos_raw, mos_valid, options);
  return stream.str();
}

std::string format_chinese_fault(
  const PackState & state, const std::uint8_t address, const double now_s, const double timeout_s)
{
  std::ostringstream stream;
  stream << "电池地址=0x" << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<unsigned int>(address) << std::dec << std::setfill(' ') << "；故障=";
  if (!state.fault_frame_s) {
    stream << "未收到故障状态";
    return stream.str();
  }
  if (!fresh(state.fault_frame_s, now_s, timeout_s)) {
    stream << "故障状态通信超时";
    return stream.str();
  }
  bool found = false;
  for (std::size_t byte = 0U; byte < kFaultNames.size(); ++byte) {
    for (std::size_t bit = 0U; bit < kFaultNames[byte].size(); ++bit) {
      if (
        !kFaultNames[byte][bit].empty() &&
        (state.fault_bytes[byte] & static_cast<std::uint8_t>(1U << bit)) != 0U) {
        if (found) {
          stream << "、";
        }
        stream << kFaultNames[byte][bit];
        found = true;
      }
    }
  }
  if (state.fault_bytes[7U] != 0U) {
    if (found) {
      stream << "、";
    }
    stream << "故障码" << static_cast<unsigned int>(state.fault_bytes[7U]);
    found = true;
  }
  if (!found) {
    stream << "无故障";
  }
  return stream.str();
}

}  // namespace bms_node
