#include "bms_node/dual_battery_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace bms_node
{

DualBatteryController::DualBatteryController(const DischargeSettings settings)
: settings_{settings}
{
  if (!std::isfinite(settings_.status_timeout_s) || settings_.status_timeout_s <= 0.0 ||
    !std::isfinite(settings_.command_timeout_s) || settings_.command_timeout_s <= 0.0 ||
    !std::isfinite(settings_.observation_s) || settings_.observation_s <= 0.0 ||
    !std::isfinite(settings_.stable_s) || settings_.stable_s <= 0.0 ||
    !std::isfinite(settings_.max_voltage_delta_v) || settings_.max_voltage_delta_v < 0.0 ||
    !std::isfinite(settings_.trip_voltage_delta_v) ||
    settings_.trip_voltage_delta_v < settings_.max_voltage_delta_v ||
    !std::isfinite(settings_.max_abs_current_a) || settings_.max_abs_current_a < 0.0 ||
    !std::isfinite(settings_.min_cell_voltage_v) ||
    !std::isfinite(settings_.max_cell_voltage_v) ||
    settings_.min_cell_voltage_v <= 0.0 ||
    settings_.max_cell_voltage_v <= settings_.min_cell_voltage_v ||
    !std::isfinite(settings_.min_cell_temperature_c) ||
    !std::isfinite(settings_.max_cell_temperature_c) ||
    settings_.max_cell_temperature_c <= settings_.min_cell_temperature_c ||
    settings_.mos_on_raw == settings_.mos_off_raw)
  {
    throw std::invalid_argument{"invalid dual battery discharge settings"};
  }
}

bool DualBatteryController::feedback_fresh(
  const std::optional<double> & stamp, const double now_s) const noexcept
{
  return stamp && std::isfinite(now_s) && std::isfinite(*stamp) &&
         now_s >= *stamp && now_s - *stamp <= settings_.status_timeout_s;
}

bool DualBatteryController::healthy(const PackState & pack, const double now_s) const noexcept
{
  if (!pack.status_fresh(now_s, settings_.status_timeout_s) ||
    !pack.cells_fresh(now_s, settings_.status_timeout_s) ||
    !feedback_fresh(pack.temperature_frame_s, now_s) ||
    !pack.max_cell_temperature_c || !pack.min_cell_temperature_c ||
    *pack.min_cell_temperature_c < settings_.min_cell_temperature_c ||
    *pack.max_cell_temperature_c > settings_.max_cell_temperature_c ||
    !pack.cell_extremes_s ||
    !feedback_fresh(pack.cell_extremes_s, now_s) ||
    !pack.min_cell_voltage_v || !pack.max_cell_voltage_v ||
    *pack.min_cell_voltage_v < settings_.min_cell_voltage_v ||
    *pack.max_cell_voltage_v > settings_.max_cell_voltage_v)
  {
    return false;
  }
  for (std::size_t index = 0U; index < *pack.cell_count; ++index) {
    if (*pack.cell_voltage_v[index] < settings_.min_cell_voltage_v ||
      *pack.cell_voltage_v[index] > settings_.max_cell_voltage_v)
    {
      return false;
    }
  }
  return
         !pack.has_fault() && pack.sample.voltage_v && pack.sample.current_a &&
         std::isfinite(*pack.sample.voltage_v) && std::isfinite(*pack.sample.current_a) &&
         pack.operating_state_raw && *pack.operating_state_raw != 1U &&
         pack.charger_connected_raw && *pack.charger_connected_raw == 0U &&
         feedback_fresh(pack.pack_info_s, now_s);
}

bool DualBatteryController::join_conditions(const DischargeInputs & inputs) const noexcept
{
  const auto & first = inputs.primary;
  const auto & second = inputs.secondary;
  return inputs.control_authorized && inputs.loads_stopped &&
         feedback_fresh(inputs.loads_stopped_s, inputs.now_s) &&
         inputs.relay_closed == false &&
         feedback_fresh(inputs.relay_feedback_s, inputs.now_s) &&
         healthy(first, inputs.now_s) && healthy(second, inputs.now_s) &&
         first.discharge_mos_raw == settings_.mos_on_raw &&
         (second.discharge_mos_raw == settings_.mos_off_raw ||
         second.discharge_mos_raw == settings_.mos_on_raw) &&
         std::abs(*first.sample.voltage_v - *second.sample.voltage_v) <=
         settings_.max_voltage_delta_v &&
         std::abs(*first.sample.current_a) <= settings_.max_abs_current_a &&
         std::abs(*second.sample.current_a) <= settings_.max_abs_current_a;
}

void DualBatteryController::latch_fault(const bool primary, const bool secondary) noexcept
{
  primary_fault_ = primary_fault_ || primary;
  secondary_fault_ = secondary_fault_ || secondary;
  if (phase_ != DischargePhase::kStoppingLoads &&
    phase_ != DischargePhase::kStoppingSecondary &&
    phase_ != DischargePhase::kOpeningRelay &&
    phase_ != DischargePhase::kStoppingPrimary &&
    phase_ != DischargePhase::kFaultLatched)
  {
    phase_ = DischargePhase::kStoppingLoads;
  }
}

DischargeAction DualBatteryController::update(const DischargeInputs & inputs)
{
  if (!std::isfinite(inputs.now_s)) {
    latch_fault(true, true);
    return DischargeAction::kStopLoads;
  }

  const bool primary_ok = healthy(inputs.primary, inputs.now_s);
  const bool secondary_ok = healthy(inputs.secondary, inputs.now_s);
  primary_healthy_seen_ = primary_healthy_seen_ || primary_ok;
  secondary_healthy_seen_ = secondary_healthy_seen_ || secondary_ok;
  const bool primary_fault = inputs.primary.has_fault() ||
    (primary_healthy_seen_ && !primary_ok);
  bool secondary_fault = inputs.secondary.has_fault() ||
    (secondary_healthy_seen_ && !secondary_ok);
  const bool joining = phase_ == DischargePhase::kWaitingForAck ||
    phase_ == DischargePhase::kWaitingForMos ||
    phase_ == DischargePhase::kWaitingForRelayClosed ||
    phase_ == DischargePhase::kObservingSecondary;
  const bool connected = phase_ == DischargePhase::kObservingSecondary ||
    phase_ == DischargePhase::kRunning;
  if (joining && primary_ok && secondary_ok &&
    (std::abs(*inputs.primary.sample.voltage_v - *inputs.secondary.sample.voltage_v) >
    settings_.max_voltage_delta_v ||
    std::abs(*inputs.primary.sample.current_a) > settings_.max_abs_current_a ||
    std::abs(*inputs.secondary.sample.current_a) > settings_.max_abs_current_a ||
    !inputs.loads_stopped ||
    !feedback_fresh(inputs.loads_stopped_s, inputs.now_s)))
  {
    secondary_fault = true;
  }
  if ((joining || connected) && !inputs.control_authorized) {
    secondary_fault = true;
  }
  if (connected &&
    (inputs.relay_closed != true ||
    !feedback_fresh(inputs.relay_feedback_s, inputs.now_s) ||
    inputs.secondary.discharge_mos_raw != settings_.mos_on_raw))
  {
    secondary_fault = true;
  }
  if (connected && primary_ok && secondary_ok &&
    std::abs(*inputs.primary.sample.voltage_v - *inputs.secondary.sample.voltage_v) >
    settings_.trip_voltage_delta_v)
  {
    secondary_fault = true;
  }
  if (primary_fault || secondary_fault) {
    latch_fault(primary_fault, secondary_fault);
  }

  switch (phase_) {
    case DischargePhase::kWaitingForRelayOpen:
      if (inputs.relay_closed == false &&
        feedback_fresh(inputs.relay_feedback_s, inputs.now_s))
      {
        phase_ = DischargePhase::kWaitingForPrimary;
        return DischargeAction::kNone;
      }
      if (inputs.relay_closed == true &&
        feedback_fresh(inputs.relay_feedback_s, inputs.now_s))
      {
        latch_fault(false, true);
      }
      return DischargeAction::kStopLoads;
    case DischargePhase::kWaitingForPrimary:
      if (primary_ok && inputs.primary.discharge_mos_raw == settings_.mos_on_raw) {
        phase_ = DischargePhase::kWaitingForSecondary;
      }
      return DischargeAction::kNone;
    case DischargePhase::kWaitingForSecondary:
      if (secondary_ok) {
        phase_ = DischargePhase::kWaitingForJoinConditions;
      }
      return DischargeAction::kNone;
    case DischargePhase::kWaitingForJoinConditions:
      if (!join_conditions(inputs)) {
        stable_since_s_.reset();
        return DischargeAction::kNone;
      }
      if (!stable_since_s_) {
        stable_since_s_ = inputs.now_s;
        return DischargeAction::kNone;
      }
      if (inputs.now_s - *stable_since_s_ < settings_.stable_s) {
        return DischargeAction::kNone;
      }
      command_s_ = inputs.now_s;
      if (inputs.secondary.discharge_mos_raw == settings_.mos_off_raw) {
        phase_ = DischargePhase::kWaitingForAck;
        return DischargeAction::kEnableSecondary;
      }
      phase_ = DischargePhase::kWaitingForRelayClosed;
      return DischargeAction::kCloseRelay;
    case DischargePhase::kWaitingForAck:
      if (inputs.secondary.discharge_ack_s && command_s_ &&
        *inputs.secondary.discharge_ack_s > *command_s_ &&
        inputs.secondary.discharge_ack_value == 1U)
      {
        phase_ = DischargePhase::kWaitingForMos;
      } else if (command_s_ && inputs.now_s - *command_s_ > settings_.command_timeout_s) {
        latch_fault(false, true);
        return DischargeAction::kStopLoads;
      }
      return DischargeAction::kNone;
    case DischargePhase::kWaitingForMos:
      if (inputs.secondary.mos_frame_s && command_s_ &&
        *inputs.secondary.mos_frame_s > *command_s_ &&
        inputs.secondary.discharge_mos_raw == settings_.mos_on_raw)
      {
        command_s_ = inputs.now_s;
        phase_ = DischargePhase::kWaitingForRelayClosed;
        return DischargeAction::kCloseRelay;
      }
      if (command_s_ && inputs.now_s - *command_s_ > settings_.command_timeout_s) {
        latch_fault(false, true);
        return DischargeAction::kStopLoads;
      }
      return DischargeAction::kNone;
    case DischargePhase::kWaitingForRelayClosed:
      if (command_s_ && inputs.relay_closed == true &&
        feedback_fresh(inputs.relay_feedback_s, inputs.now_s) &&
        inputs.relay_feedback_s && *inputs.relay_feedback_s > *command_s_)
      {
        observation_s_ = inputs.now_s;
        phase_ = DischargePhase::kObservingSecondary;
      } else if (command_s_ && inputs.now_s - *command_s_ > settings_.command_timeout_s) {
        latch_fault(false, true);
        return DischargeAction::kStopLoads;
      }
      return DischargeAction::kNone;
    case DischargePhase::kObservingSecondary:
      if (observation_s_ && inputs.now_s - *observation_s_ >= settings_.observation_s) {
        phase_ = DischargePhase::kRunning;
      }
      return DischargeAction::kNone;
    case DischargePhase::kRunning:
      return DischargeAction::kNone;
    case DischargePhase::kStoppingLoads:
      if (!inputs.loads_stopped ||
        !feedback_fresh(inputs.loads_stopped_s, inputs.now_s) ||
        !inputs.primary.sample.is_fresh(inputs.now_s, settings_.status_timeout_s) ||
        !inputs.secondary.sample.is_fresh(inputs.now_s, settings_.status_timeout_s) ||
        !inputs.primary.sample.current_a || !inputs.secondary.sample.current_a ||
        std::abs(*inputs.primary.sample.current_a) > settings_.max_abs_current_a ||
        std::abs(*inputs.secondary.sample.current_a) > settings_.max_abs_current_a)
      {
        return DischargeAction::kStopLoads;
      }
      if (secondary_fault_ && !secondary_isolation_attempted_) {
        phase_ = DischargePhase::kStoppingSecondary;
        secondary_isolation_attempted_ = true;
        command_s_ = inputs.now_s;
        return DischargeAction::kDisableSecondary;
      }
      if (primary_fault_ && !primary_isolation_attempted_) {
        phase_ = DischargePhase::kStoppingPrimary;
        primary_isolation_attempted_ = true;
        command_s_ = inputs.now_s;
        return DischargeAction::kDisablePrimary;
      }
      phase_ = DischargePhase::kFaultLatched;
      return DischargeAction::kNone;
    case DischargePhase::kStoppingSecondary:
      if (!inputs.secondary.mos_frame_s || !command_s_ ||
        *inputs.secondary.mos_frame_s <= *command_s_ ||
        inputs.secondary.discharge_mos_raw != settings_.mos_off_raw)
      {
        if (command_s_ && inputs.now_s - *command_s_ <= settings_.command_timeout_s) {
          return DischargeAction::kDisableSecondary;
        }
      }
      if (!inputs.loads_stopped ||
        !feedback_fresh(inputs.loads_stopped_s, inputs.now_s) ||
        !inputs.primary.sample.is_fresh(inputs.now_s, settings_.status_timeout_s) ||
        !inputs.secondary.sample.is_fresh(inputs.now_s, settings_.status_timeout_s) ||
        !inputs.primary.sample.current_a || !inputs.secondary.sample.current_a ||
        std::abs(*inputs.primary.sample.current_a) > settings_.max_abs_current_a ||
        std::abs(*inputs.secondary.sample.current_a) > settings_.max_abs_current_a)
      {
        return DischargeAction::kStopLoads;
      }
      phase_ = DischargePhase::kOpeningRelay;
      command_s_ = inputs.now_s;
      return DischargeAction::kOpenRelay;
    case DischargePhase::kOpeningRelay:
      if (inputs.relay_closed == false &&
        feedback_fresh(inputs.relay_feedback_s, inputs.now_s))
      {
        if (primary_fault_ && !primary_isolation_attempted_) {
          phase_ = DischargePhase::kStoppingPrimary;
          primary_isolation_attempted_ = true;
          command_s_ = inputs.now_s;
          return DischargeAction::kDisablePrimary;
        }
        phase_ = DischargePhase::kFaultLatched;
        return DischargeAction::kNone;
      }
      if (command_s_ && inputs.now_s - *command_s_ > settings_.command_timeout_s) {
        if (primary_fault_ && !primary_isolation_attempted_) {
          phase_ = DischargePhase::kStoppingPrimary;
          primary_isolation_attempted_ = true;
          command_s_ = inputs.now_s;
          return DischargeAction::kDisablePrimary;
        }
        phase_ = DischargePhase::kFaultLatched;
        return DischargeAction::kNone;
      }
      return DischargeAction::kOpenRelay;
    case DischargePhase::kStoppingPrimary:
      if ((!inputs.primary.mos_frame_s || !command_s_ ||
        *inputs.primary.mos_frame_s <= *command_s_ ||
        inputs.primary.discharge_mos_raw != settings_.mos_off_raw) &&
        command_s_ && inputs.now_s - *command_s_ <= settings_.command_timeout_s)
      {
        return DischargeAction::kDisablePrimary;
      }
      phase_ = DischargePhase::kFaultLatched;
      return DischargeAction::kNone;
    case DischargePhase::kFaultLatched:
      if ((primary_fault_ && !primary_isolation_attempted_) ||
        (secondary_fault_ && !secondary_isolation_attempted_))
      {
        phase_ = DischargePhase::kStoppingLoads;
        return DischargeAction::kStopLoads;
      }
      return secondary_fault_ ? DischargeAction::kOpenRelay : DischargeAction::kNone;
  }
  return DischargeAction::kNone;
}

}  // namespace bms_node
