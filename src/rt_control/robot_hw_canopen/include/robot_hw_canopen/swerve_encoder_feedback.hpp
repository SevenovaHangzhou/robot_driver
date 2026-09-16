#ifndef ROBOT_HW_CANOPEN__SWERVE_ENCODER_FEEDBACK_HPP_
#define ROBOT_HW_CANOPEN__SWERVE_ENCODER_FEEDBACK_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace robot_hw_canopen
{

inline constexpr std::size_t kSwerveEncoderCount{4U};
inline constexpr std::uint16_t kSwerveEncoderPositionIndex{0x6004U};
inline constexpr std::uint8_t kSwerveEncoderPositionSubindex{0U};

struct SwerveEncoderAxisConfig
{
  std::uint8_t node_id{0U};
  std::uint32_t counts_per_revolution{0U};
  std::uint32_t ring_gear_teeth{0U};
  std::uint32_t pinion_gear_teeth{0U};
  int direction{0};
  double installation_offset_rad{0.0};
};

struct SwerveEncoderSample
{
  double position_rad{0.0};
  double feedback_age_ms{0.0};
  bool valid{false};
};

class SwerveEncoderFeedback final
{
public:
  explicit SwerveEncoderFeedback(
    std::array<SwerveEncoderAxisConfig, kSwerveEncoderCount> config);

  SwerveEncoderFeedback(const SwerveEncoderFeedback &) = delete;
  SwerveEncoderFeedback & operator=(const SwerveEncoderFeedback &) = delete;
  SwerveEncoderFeedback(SwerveEncoderFeedback &&) = delete;
  SwerveEncoderFeedback & operator=(SwerveEncoderFeedback &&) = delete;

  [[nodiscard]] bool observe(
    std::uint8_t node_id, std::uint16_t index, std::uint8_t subindex,
    std::uint32_t raw_position, std::int64_t received_steady_ns) noexcept;

  [[nodiscard]] std::array<SwerveEncoderSample, kSwerveEncoderCount> sample(
    std::int64_t now_steady_ns) const noexcept;

private:
  struct Slot
  {
    std::atomic<std::uint64_t> sequence{0U};
    std::atomic<std::uint32_t> raw_position{0U};
    std::atomic<std::int64_t> received_steady_ns{0};
  };

  [[nodiscard]] std::size_t axis_for_node(std::uint8_t node_id) const noexcept;
  [[nodiscard]] SwerveEncoderSample sample_axis(
    std::size_t axis, std::int64_t now_steady_ns) const noexcept;

  std::array<SwerveEncoderAxisConfig, kSwerveEncoderCount> config_{};
  std::array<Slot, kSwerveEncoderCount> slots_{};
};

}  // namespace robot_hw_canopen

#endif  // ROBOT_HW_CANOPEN__SWERVE_ENCODER_FEEDBACK_HPP_
