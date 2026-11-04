#ifndef BMS_NODE__CHINESE_STATUS_HPP_
#define BMS_NODE__CHINESE_STATUS_HPP_

#include "periph_bms/protocol/protocol.hpp"

#include <cstdint>
#include <string>

namespace periph_bms
{

struct ChineseStatusOptions
{
  int mos_closed_raw{-1};
  int mos_open_raw{-1};
};

[[nodiscard]] std::string format_chinese_status(
  const PackState & state, std::uint8_t address, double now_s, double timeout_s,
  const ChineseStatusOptions & options);

[[nodiscard]] std::string format_chinese_fault(
  const PackState & state, std::uint8_t address, double now_s, double timeout_s);

}  // namespace periph_bms

#endif  // BMS_NODE__CHINESE_STATUS_HPP_
