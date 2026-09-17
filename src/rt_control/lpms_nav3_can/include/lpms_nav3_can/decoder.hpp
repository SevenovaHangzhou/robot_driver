#ifndef LPMS_NAV3_CAN__DECODER_HPP_
#define LPMS_NAV3_CAN__DECODER_HPP_

#include <array>
#include <cstdint>
#include <optional>

namespace lpms_nav3_can
{

struct CanFrame
{
  std::uint32_t id{0U};
  std::uint8_t dlc{0U};
  std::array<std::uint8_t, 8U> data{};
  bool is_extended{false};
  bool is_remote{false};
  bool is_error{false};
};

struct ImuSample
{
  std::array<double, 3U> linear_acceleration_mps2{};
  std::array<double, 3U> angular_velocity_radps{};
  std::array<double, 3U> magnetic_field_t{};
  std::array<double, 4U> orientation_wxyz{};
  std::array<double, 3U> euler_rad{};
};

struct DecodeResult
{
  std::optional<ImuSample> sample;
  std::optional<std::uint8_t> heartbeat_state;
  bool recognized{false};
  bool malformed{false};
};

class Decoder
{
public:
  explicit Decoder(std::uint8_t node_id);

  DecodeResult consume(const CanFrame & frame);
  void reset() noexcept;
  [[nodiscard]] std::uint8_t node_id() const noexcept;
  [[nodiscard]] std::uint8_t pending_mask() const noexcept;

private:
  std::uint8_t node_id_;
  std::uint8_t pending_mask_{0U};
  ImuSample sample_{};
};

[[nodiscard]] ImuSample convert_to_ros_convention(
  const ImuSample & sample, bool convert_to_ros);

}  // namespace lpms_nav3_can

#endif  // LPMS_NAV3_CAN__DECODER_HPP_
