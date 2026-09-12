#include "dm_swerve_driver/canopen_encoder.hpp"

#include <algorithm>
#include <vector>

namespace dm_swerve_driver {

void CanopenEncoderClient::configure_sync()
{
  if (!is_open()) {throw CanopenEncoderError{"encoder transport is closed"};}
  std::vector<CanFrame> requests;
  for (const auto node : config_.node_ids) {
    CanFrame frame{0U, 2U, {}};
    frame.data[0] = 0x80U;  // NMT Pre-operational; addressed nodes only.
    frame.data[1] = node;
    requests.push_back(frame);
  }
  transport_->write_batch(requests);
  const auto mapping = read_object(0x1A01U, 1U);
  const auto entries = read_object(0x1A01U, 0U);
  const auto operating = read_object(0x6000U, 0U);
  for (std::size_t i{0U}; i < kSwerveModuleCount; ++i) {
    if (mapping[i] != 0x60040020U || entries[i] != 1U || (operating[i] & 0x04U) != 0U) {
      throw CanopenEncoderError{"BRT TPDO2 mapping or encoder scaling differs from configuration"};
    }
  }
  const auto download = [this](std::uint16_t index, std::uint8_t sub, std::uint16_t value,
      std::uint8_t command) {
      std::vector<CanFrame> writes;
      for (const auto node : config_.node_ids) {
        auto frame = make_canopen_sdo_upload(node, index, sub);
        frame.data[0] = command;
        frame.data[4] = static_cast<std::uint8_t>(value & 0xFFU);
        frame.data[5] = static_cast<std::uint8_t>(value >> 8U);
        writes.push_back(frame);
      }
      transport_->write_batch(writes);
      std::array<bool, kSwerveModuleCount> confirmed{};
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::microseconds{config_.startup_deadline_us};
      for (unsigned int attempt{0U}; attempt < 16U; ++attempt) {
        const auto replies = transport_->collect(kSwerveModuleCount, deadline);
        if (replies.empty()) {break;}
        for (const auto & reply : replies) {
          const auto & frame = reply.frame;
          if (frame.id <= 0x580U || frame.id > 0x5FFU || frame.length != 8U) {continue;}
          const auto module = module_for_node(static_cast<std::uint8_t>(frame.id - 0x580U));
          if (module >= kSwerveModuleCount || frame.data[1] != (index & 0xFFU) ||
            frame.data[2] != (index >> 8U) || frame.data[3] != sub) {continue;}
          if (frame.data[0] == 0x80U) {throw CanopenEncoderError{"encoder setup SDO aborted"};}
          if (frame.data[0] == 0x60U) {confirmed[module] = true;}
        }
        if (std::all_of(confirmed.begin(), confirmed.end(), [](bool v) {return v;})) {return;}
      }
      throw CanopenEncoderError{"encoder setup SDO acknowledgement missing"};
    };
  download(0x1801U, 2U, 1U, 0x2FU);
  download(0x1017U, 0U, 100U, 0x2BU);
  const auto transmission = read_object(0x1801U, 2U);
  if (!std::all_of(transmission.begin(), transmission.end(), [](auto v) {return v == 1U;})) {
    throw CanopenEncoderError{"encoder SYNC transmission type readback mismatch"};
  }
  for (auto & frame : requests) {frame.data[0] = 0x01U;}
  transport_->write_batch(requests);  // NMT Operational after acknowledged setup.
}

}  // namespace dm_swerve_driver
