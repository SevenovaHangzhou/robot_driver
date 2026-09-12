#ifndef DM_SWERVE_DRIVER__CANOPEN_ENCODER_HPP_
#define DM_SWERVE_DRIVER__CANOPEN_ENCODER_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include "dm_swerve_driver/can_transport.hpp"
#include "dm_swerve_driver/external_steering_encoder.hpp"

namespace dm_swerve_driver {

inline constexpr std::uint16_t kCanopenSyncId{0x080U};
inline constexpr std::uint16_t kCanopenTpdo2BaseId{0x280U};
inline constexpr std::uint16_t kCanopenSdoRequestBaseId{0x600U};
inline constexpr std::uint16_t kCanopenSdoResponseBaseId{0x580U};

class CanopenEncoderError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

struct EncoderCanopenConfig {
  std::array<std::uint8_t, kSwerveModuleCount> node_ids{};
  std::int64_t startup_deadline_us{0};
};

struct CanopenEncoderCycle {
  std::array<std::uint32_t, kSwerveModuleCount> positions{};
  std::array<bool, kSwerveModuleCount> received{};
  std::size_t unknown_frames{0U};
  std::size_t rejected_frames{0U};
  std::size_t duplicate_frames{0U};
  std::size_t stale_frames{0U};
  std::array<bool, kSwerveModuleCount> heartbeat_seen{};
  std::array<std::uint8_t, kSwerveModuleCount> nmt_state{};

  [[nodiscard]] bool complete() const noexcept;
};

[[nodiscard]] CanFrame make_canopen_sync() noexcept;
[[nodiscard]] CanFrame make_canopen_sdo_upload(
  std::uint8_t node_id, std::uint16_t index, std::uint8_t subindex);
[[nodiscard]] std::uint32_t decode_canopen_sdo_upload(
  const CanFrame & frame,
  std::uint8_t expected_node_id,
  std::uint16_t expected_index,
  std::uint8_t expected_subindex);

class CanopenEncoderClient final {
public:
  CanopenEncoderClient(
    EncoderCanopenConfig config,
    std::unique_ptr<CanTransport> transport);
  ~CanopenEncoderClient() noexcept;

  CanopenEncoderClient(const CanopenEncoderClient &) = delete;
  CanopenEncoderClient & operator=(const CanopenEncoderClient &) = delete;
  CanopenEncoderClient(CanopenEncoderClient &&) = delete;
  CanopenEncoderClient & operator=(CanopenEncoderClient &&) = delete;

  void open();
  void close() noexcept;
  [[nodiscard]] bool is_open() const noexcept;
  [[nodiscard]] std::array<EncoderHardwareInfo, kSwerveModuleCount>
  read_hardware_info();
  void configure_sync();
  [[nodiscard]] CanopenEncoderCycle sample(
    std::chrono::steady_clock::time_point deadline);

private:
  [[nodiscard]] std::array<std::uint32_t, kSwerveModuleCount> read_object(
    std::uint16_t index,
    std::uint8_t subindex);
  [[nodiscard]] std::size_t module_for_node(std::uint8_t node_id) const noexcept;

  EncoderCanopenConfig config_;
  std::unique_ptr<CanTransport> transport_;
  std::array<std::chrono::steady_clock::time_point, kSwerveModuleCount> heartbeat_time_{};
  std::array<bool, kSwerveModuleCount> heartbeat_seen_{};
  std::array<std::uint8_t, kSwerveModuleCount> nmt_state_{};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__CANOPEN_ENCODER_HPP_
