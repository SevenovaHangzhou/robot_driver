#include "dm_swerve_driver/kinco_ds402.hpp"

namespace dm_swerve_driver {
namespace {

constexpr std::uint16_t kDisableVoltage{0x0000U};
constexpr std::uint16_t kShutdown{0x0006U};
constexpr std::uint16_t kSwitchOn{0x0007U};
constexpr std::uint16_t kEnableOperation{0x000FU};
constexpr std::uint16_t kFaultReset{0x0080U};

[[nodiscard]] bool bit_set(std::uint16_t word, unsigned int bit) noexcept
{
  return (word & static_cast<std::uint16_t>(1U << bit)) != 0U;
}

}  // namespace

Ds402State decode_ds402_state(std::uint16_t status_word) noexcept
{
  if ((status_word & 0x004FU) == 0x0000U) {
    return Ds402State::not_ready_to_switch_on;
  }
  if ((status_word & 0x004FU) == 0x0040U) {
    return Ds402State::switch_on_disabled;
  }
  if ((status_word & 0x006FU) == 0x0021U) {
    return Ds402State::ready_to_switch_on;
  }
  if ((status_word & 0x006FU) == 0x0023U) {
    return Ds402State::switched_on;
  }
  if ((status_word & 0x006FU) == 0x0027U) {
    return Ds402State::operation_enabled;
  }
  if ((status_word & 0x006FU) == 0x0007U) {
    return Ds402State::quick_stop_active;
  }
  if ((status_word & 0x004FU) == 0x000FU) {
    return Ds402State::fault_reaction_active;
  }
  if ((status_word & 0x004FU) == 0x0008U) {
    return Ds402State::fault;
  }
  return Ds402State::unknown;
}

std::uint16_t ds402_enable_control_word(Ds402State state) noexcept
{
  switch (state) {
    case Ds402State::switch_on_disabled:
      return kShutdown;
    case Ds402State::ready_to_switch_on:
      return kSwitchOn;
    case Ds402State::switched_on:
    case Ds402State::operation_enabled:
    case Ds402State::quick_stop_active:
      return kEnableOperation;
    case Ds402State::unknown:
    case Ds402State::not_ready_to_switch_on:
    case Ds402State::fault_reaction_active:
    case Ds402State::fault:
      return kDisableVoltage;
  }
  return kDisableVoltage;
}

KincoFaultReport classify_kinco_error_word(std::uint16_t error_word) noexcept
{
  KincoFaultReport report;
  report.encoder = (error_word & 0x400EU) != 0U;
  report.over_temperature = (error_word & 0x2010U) != 0U;
  report.over_voltage = bit_set(error_word, 5U);
  report.under_voltage = (error_word & 0x0440U) != 0U;
  report.over_current = bit_set(error_word, 7U);
  report.braking_resistor = bit_set(error_word, 8U);
  report.following_error = bit_set(error_word, 9U);
  report.overload = bit_set(error_word, 11U);
  report.internal_or_configuration = (error_word & 0x9001U) != 0U;

  constexpr std::uint16_t recoverable_mask{0x0440U};
  const std::uint16_t latching_bits{
    static_cast<std::uint16_t>(error_word & static_cast<std::uint16_t>(~recoverable_mask))};
  if (error_word == 0U) {
    report.disposition = FaultDisposition::none;
  } else if (latching_bits == 0U) {
    report.disposition = FaultDisposition::recoverable;
  } else {
    report.disposition = FaultDisposition::latch;
  }
  return report;
}

void Ds402FaultResetSequence::request() noexcept
{
  if (phase_ == Phase::idle) {
    phase_ = Phase::low_before_pulse;
  }
}

std::uint16_t Ds402FaultResetSequence::next_control_word(Ds402State state) noexcept
{
  if (state != Ds402State::fault) {
    phase_ = Phase::idle;
    return ds402_enable_control_word(state);
  }
  switch (phase_) {
    case Phase::idle:
      return kDisableVoltage;
    case Phase::low_before_pulse:
      phase_ = Phase::pulse;
      return kDisableVoltage;
    case Phase::pulse:
      phase_ = Phase::low_after_pulse;
      return kFaultReset;
    case Phase::low_after_pulse:
      phase_ = Phase::idle;
      return kDisableVoltage;
  }
  return kDisableVoltage;
}

bool Ds402FaultResetSequence::active() const noexcept
{
  return phase_ != Phase::idle;
}

}  // namespace dm_swerve_driver
