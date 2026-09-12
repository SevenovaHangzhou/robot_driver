#include "dm_swerve_driver/canopen_encoder.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace dm_swerve_driver {
namespace {

constexpr std::uint8_t kSdoUploadRequest{0x40U};
constexpr std::uint8_t kSdoAbort{0x80U};

void validate_node_id(std::uint8_t node_id)
{
  if (node_id == 0U || node_id > 127U) {
    throw std::invalid_argument{"CANopen node ID must be in [1, 127]"};
  }
}

[[nodiscard]] std::uint16_t frame_index(const CanFrame & frame) noexcept
{
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(frame.data[1]) |
    static_cast<std::uint16_t>(static_cast<std::uint16_t>(frame.data[2]) << 8U));
}

[[nodiscard]] std::uint32_t little_endian_value(
  const CanFrame & frame, std::size_t size) noexcept
{
  std::uint32_t value{0U};
  for (std::size_t byte{0U}; byte < size; ++byte) {
    value |= static_cast<std::uint32_t>(frame.data[4U + byte]) << (8U * byte);
  }
  return value;
}

[[nodiscard]] std::size_t expedited_size(std::uint8_t command)
{
  if ((command & 0xE0U) != 0x40U || (command & 0x02U) == 0U) {
    throw CanopenEncoderError{"CANopen SDO response is not an expedited upload"};
  }
  if ((command & 0x01U) == 0U) {
    throw CanopenEncoderError{"CANopen SDO response does not declare its size"};
  }
  const std::size_t unused{static_cast<std::size_t>((command >> 2U) & 0x03U)};
  return 4U - unused;
}

[[nodiscard]] std::uint8_t node_from_response_id(std::uint16_t id) noexcept
{
  if (id <= kCanopenSdoResponseBaseId ||
    id > kCanopenSdoResponseBaseId + 127U)
  {
    return 0U;
  }
  return static_cast<std::uint8_t>(id - kCanopenSdoResponseBaseId);
}

[[nodiscard]] std::uint8_t node_from_tpdo2_id(std::uint16_t id) noexcept
{
  if (id <= kCanopenTpdo2BaseId || id > kCanopenTpdo2BaseId + 127U) {
    return 0U;
  }
  return static_cast<std::uint8_t>(id - kCanopenTpdo2BaseId);
}

}  // namespace

bool CanopenEncoderCycle::complete() const noexcept
{
  return std::all_of(received.begin(), received.end(), [](bool value) {return value;});
}

CanFrame make_canopen_sync() noexcept
{
  CanFrame frame;
  frame.id = kCanopenSyncId;
  frame.length = 0U;
  return frame;
}

CanFrame make_canopen_sdo_upload(
  std::uint8_t node_id, std::uint16_t index, std::uint8_t subindex)
{
  validate_node_id(node_id);
  CanFrame frame;
  frame.id = static_cast<std::uint16_t>(kCanopenSdoRequestBaseId + node_id);
  frame.data[0] = kSdoUploadRequest;
  frame.data[1] = static_cast<std::uint8_t>(index & 0xFFU);
  frame.data[2] = static_cast<std::uint8_t>((index >> 8U) & 0xFFU);
  frame.data[3] = subindex;
  return frame;
}

std::uint32_t decode_canopen_sdo_upload(
  const CanFrame & frame,
  std::uint8_t expected_node_id,
  std::uint16_t expected_index,
  std::uint8_t expected_subindex)
{
  validate_node_id(expected_node_id);
  if (frame.id != kCanopenSdoResponseBaseId + expected_node_id ||
    frame.length != kCanPayloadSize || frame_index(frame) != expected_index ||
    frame.data[3] != expected_subindex)
  {
    throw CanopenEncoderError{"CANopen SDO response does not match the request"};
  }
  if (frame.data[0] == kSdoAbort) {
    std::ostringstream message;
    message << "CANopen SDO upload aborted with code 0x" << std::hex <<
      little_endian_value(frame, 4U);
    throw CanopenEncoderError{message.str()};
  }
  return little_endian_value(frame, expedited_size(frame.data[0]));
}

CanopenEncoderClient::CanopenEncoderClient(
  EncoderCanopenConfig config,
  std::unique_ptr<CanTransport> transport)
: config_{std::move(config)}, transport_{std::move(transport)}
{
  if (transport_ == nullptr) {
    throw std::invalid_argument{"CANopen encoder client requires a transport"};
  }
  if (config_.startup_deadline_us <= 0) {
    throw std::invalid_argument{"CANopen startup deadline must be positive"};
  }
  auto sorted = config_.node_ids;
  for (const auto node_id : sorted) {
    validate_node_id(node_id);
  }
  std::sort(sorted.begin(), sorted.end());
  if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) {
    throw std::invalid_argument{"CANopen encoder node IDs must be unique"};
  }
}

CanopenEncoderClient::~CanopenEncoderClient() noexcept
{
  close();
}

void CanopenEncoderClient::open()
{
  transport_->open();
}

void CanopenEncoderClient::close() noexcept
{
  transport_->close();
}

bool CanopenEncoderClient::is_open() const noexcept
{
  return transport_->is_open();
}

std::array<EncoderHardwareInfo, kSwerveModuleCount>
CanopenEncoderClient::read_hardware_info()
{
  const auto resolutions = read_object(0x6501U, 0U);
  const auto revolutions = read_object(0x6502U, 0U);
  std::array<EncoderHardwareInfo, kSwerveModuleCount> result{};
  for (std::size_t index{0U}; index < result.size(); ++index) {
    result[index] = EncoderHardwareInfo{resolutions[index], revolutions[index]};
  }
  return result;
}

CanopenEncoderCycle CanopenEncoderClient::sample(
  std::chrono::steady_clock::time_point deadline)
{
  if (!transport_->is_open()) {
    throw CanopenEncoderError{"CANopen encoder transport is closed"};
  }
  CanopenEncoderCycle result;
  const auto sample_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch());
  transport_->write_batch({make_canopen_sync()});
  for (unsigned int collection{0U}; collection < 16U && !result.complete(); ++collection) {
  const auto frames = transport_->collect(kSwerveModuleCount, deadline);
  if (frames.empty()) {break;}
  for (const auto & received : frames) {
    const auto & frame = received.frame;
    if (frame.id > 0x700U && frame.id <= 0x77FU && frame.length == 1U) {
      const auto index = module_for_node(static_cast<std::uint8_t>(frame.id - 0x700U));
      if (index < kSwerveModuleCount) {
        const auto age = received.kernel_timestamp == std::chrono::nanoseconds{} ?
          std::chrono::nanoseconds{} : std::max(std::chrono::nanoseconds{},
          sample_time - received.kernel_timestamp);
        heartbeat_time_[index] = std::chrono::steady_clock::now() - age;
        heartbeat_seen_[index] = true;
        nmt_state_[index] = frame.data[0];
        continue;
      }
    }
    if (received.kernel_timestamp != std::chrono::nanoseconds{} &&
      received.kernel_timestamp < sample_time) {
      ++result.stale_frames;
      continue;
    }
    const std::uint8_t node_id{node_from_tpdo2_id(frame.id)};
    const std::size_t module{module_for_node(node_id)};
    if (module == kSwerveModuleCount) {
      ++result.unknown_frames;
      continue;
    }
    if (frame.length != 4U) {
      ++result.rejected_frames;
      continue;
    }
    if (result.received[module]) {
      ++result.duplicate_frames;
      continue;
    }
    std::uint32_t position{0U};
    for (std::size_t byte{0U}; byte < 4U; ++byte) {
      position |= static_cast<std::uint32_t>(frame.data[byte]) << (8U * byte);
    }
    result.positions[module] = position;
    result.received[module] = true;
  }
  }
  for (std::size_t i{0U}; i < kSwerveModuleCount; ++i) {
    result.heartbeat_seen[i] = heartbeat_seen_[i] &&
      std::chrono::steady_clock::now() - heartbeat_time_[i] < std::chrono::milliseconds{300};
    result.nmt_state[i] = nmt_state_[i];
  }
  return result;
}

std::array<std::uint32_t, kSwerveModuleCount> CanopenEncoderClient::read_object(
  std::uint16_t index,
  std::uint8_t subindex)
{
  if (!transport_->is_open()) {
    throw CanopenEncoderError{"CANopen encoder transport is closed"};
  }
  std::vector<CanFrame> requests;
  requests.reserve(kSwerveModuleCount);
  for (const auto node_id : config_.node_ids) {
    requests.push_back(make_canopen_sdo_upload(node_id, index, subindex));
  }
  const auto request_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch());
  transport_->write_batch(requests);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::microseconds{config_.startup_deadline_us};
  std::array<std::uint32_t, kSwerveModuleCount> result{};
  std::array<bool, kSwerveModuleCount> received{};
  for (unsigned int collection{0U}; collection < 16U; ++collection) {
  const auto frames = transport_->collect(kSwerveModuleCount, deadline);
  if (frames.empty()) {break;}
  for (const auto & item : frames) {
    if (item.kernel_timestamp != std::chrono::nanoseconds{} &&
      item.kernel_timestamp < request_time) {continue;}
    const std::uint8_t node_id{node_from_response_id(item.frame.id)};
    const std::size_t module{module_for_node(node_id)};
    if (module == kSwerveModuleCount || received[module]) {
      continue;
    }
    if (item.frame.length != 8U || frame_index(item.frame) != index ||
      item.frame.data[3] != subindex) {continue;}
    result[module] = decode_canopen_sdo_upload(
      item.frame, node_id, index, subindex);
    received[module] = true;
  }
  if (std::all_of(received.begin(), received.end(), [](bool value) {return value;})) {break;}
  }
  if (!std::all_of(received.begin(), received.end(), [](bool value) {return value;})) {
    throw CanopenEncoderError{"CANopen encoder startup object read is incomplete"};
  }
  return result;
}

std::size_t CanopenEncoderClient::module_for_node(std::uint8_t node_id) const noexcept
{
  const auto found = std::find(config_.node_ids.begin(), config_.node_ids.end(), node_id);
  return found == config_.node_ids.end() ? kSwerveModuleCount :
         static_cast<std::size_t>(found - config_.node_ids.begin());
}

}  // namespace dm_swerve_driver
