#include "swerve_driver/swerve_controller.hpp"

#include <cmath>
#include <limits>

namespace swerve_driver
{
const char * SwerveController::mode_result_text(ModeResult result) noexcept
{
  switch (result) {
    case ModeResult::Applied:
      return "Mode confirmed by control cycle; send a new velocity command";
    case ModeResult::NotReady:
      return "Controller is not active or has a latched fault";
    case ModeResult::FeedbackInvalid:
      return "Required module feedback is invalid or stale";
    case ModeResult::NotStopped:
      return "Wheel references/feedback have not satisfied the stationary window";
    case ModeResult::Expired:
      return "Request expired; keep zero velocity and check the mode topic";
    case ModeResult::NotNavigation:
      return "Chassis is owned by a non-navigation execution mode";
    case ModeResult::FreshZeroRequired:
      return "A fresh explicit zero velocity command is required";
    case ModeResult::InvalidParameters:
      return "Alignment parameters are invalid";
  }
  return "Unknown mode-switch result";
}

void SwerveController::request_drive_mode(
  ModeServer::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
  ModeService::Request::SharedPtr request)
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  ModeService::Response rejected;
  auto reject = [&](const char * reason) {
    rejected.success = false;
    rejected.message = reason;
    try {
      service->send_response(*header, rejected);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_node()->get_logger(), "mode rejection delivery failed: %s", error.what());
    }
  };
  if (mode_response_) {
    reject("A mode request is already pending; no request queue");
    return;
  }
  if (!active_.load() || fault_latched_.load() || !core_) {
    reject(mode_result_text(ModeResult::NotReady));
    return;
  }
  ModeRequest candidate;
  candidate.align_first = request->data;
  candidate.threshold =
    get_node()
      ->get_parameter(candidate.align_first ? "align_first_threshold" : "alignment_threshold")
      .as_double();
  candidate.release = candidate.align_first
                        ? get_node()->get_parameter("align_first_release").as_double()
                        : std::numeric_limits<double>::quiet_NaN();
  if (
    !std::isfinite(candidate.threshold) || candidate.threshold <= 0.0 ||
    candidate.threshold > kPi / 2.0 ||
    (candidate.align_first && (!std::isfinite(candidate.release) || candidate.release <= 0.0 ||
                               candidate.release > candidate.threshold))) {
    reject(mode_result_text(ModeResult::InvalidParameters));
    return;
  }
  candidate.sequence = ++mode_sequence_;
  candidate.generation = generation_;
  // Transport response deadline, not a stop timeout. Moving requests are rejected on the
  // next RT cycle, never held until the chassis eventually stops.
  candidate.deadline_ns = steady_now() + 1000000000LL;
  mode_response_ = ModeResponse{service, *header, candidate};
  mode_request_buffer_.writeFromNonRT(candidate);
  mode_pending_.store(candidate.sequence);
  // Deferred service: leave the executor free for cmd_vel and lifecycle callbacks.
}

void SwerveController::mode_timer()
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  const auto now = steady_now();
  if (mode_response_) {
    const auto & pending = mode_response_->request;
    const bool complete = mode_completed_.load() == pending.sequence;
    const bool expired = now >= pending.deadline_ns;
    const bool inactive = !active_.load() || pending.generation != generation_;
    if (complete || expired || inactive) {
      ModeService::Response response;
      if (complete) {
        const auto result = mode_result_.load();
        response.success = result == ModeResult::Applied;
        response.message = mode_result_text(result);
      } else {
        auto sequence = pending.sequence;
        const bool canceled = mode_pending_.compare_exchange_strong(sequence, 0);
        response.success = false;
        response.message =
          canceled
            ? "Request canceled before execution; keep zero velocity"
            : "Result not confirmed; keep zero velocity and check /rt_control/chassis/align_first";
      }
      try {
        mode_response_->service->send_response(mode_response_->header, response);
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_node()->get_logger(), "mode response delivery failed: %s", error.what());
      }
      mode_response_.reset();
    }
  }
  const bool mode = align_first_.load();
  if (
    mode_publisher_ &&
    (!published_mode_ || *published_mode_ != mode || now - mode_published_at_ns_ >= 1000000000LL)) {
    std_msgs::msg::Bool message;
    message.data = mode;
    mode_publisher_->publish(message);
    published_mode_ = mode;
    mode_published_at_ns_ = now;
  }
}

void SwerveController::process_drive_mode(
  const ModuleFeedbackArray & feedback, const Command & command, bool command_fresh,
  bool navigation, int64_t now)
{
  const bool valid = ready_.load() && !fault_latched_.load() &&
                     core_->mode_switch_feedback_ready(feedback) &&
                     last_output_.status == ControlStatus::running;
  const bool fresh_zero = command_fresh && command.received_ns > block_before_ns_ &&
                          command.speeds.vx_mps == 0.0 && command.speeds.vy_mps == 0.0 &&
                          command.speeds.omega_radps == 0.0;
  bool stopped = valid && fresh_zero && navigation;
  for (size_t i = 0; i < 4; ++i) {
    const double speed = feedback[i].wheel_velocity * config_.wheel_radius[i];
    stopped = stopped && std::isfinite(speed) && std::abs(speed) <= fault_stop_speed_ &&
              last_output_.drive_velocity[i] == 0.0;
  }
  if (!stopped) {
    mode_stationary_since_ns_ = 0;
  } else if (mode_stationary_since_ns_ == 0) {
    mode_stationary_since_ns_ = now;
  }

  const auto sequence = mode_pending_.load();
  if (sequence == 0) {
    return;
  }
  const auto request = *mode_request_buffer_.readFromRT();
  if (request.sequence != sequence) {
    return;
  }  // writer has not swapped buffers yet
  ModeResult result = ModeResult::Applied;
  if (!active_.load() || request.generation != generation_ || fault_latched_.load()) {
    result = ModeResult::NotReady;
  } else if (now >= request.deadline_ns) {
    result = ModeResult::Expired;
  } else if (!navigation) {
    result = ModeResult::NotNavigation;
  } else if (!valid) {
    result = ModeResult::FeedbackInvalid;
  } else if (!fresh_zero) {
    result = ModeResult::FreshZeroRequired;
  } else if (
    !stopped || static_cast<double>(now - mode_stationary_since_ns_) * 1e-9 < fault_stop_hold_) {
    result = ModeResult::NotStopped;
  }
  auto expected = sequence;
  if (!mode_pending_.compare_exchange_strong(expected, 0)) {
    return;
  }
  if (result == ModeResult::Applied) {
    if (core_->switch_alignment_policy(
          feedback, fault_stop_speed_, request.threshold, request.release)) {
      align_first_.store(request.align_first);
      // Exclude commands already received before the switch. No cached nonzero command can
      // restart the wheels when a gate is released; navigation must send a new request.
      block_before_ns_ = now;
      mode_stationary_since_ns_ = 0;
    } else {
      result = ModeResult::NotStopped;
    }
  }
  mode_result_.store(result);
  mode_completed_.store(sequence);
}
}  // namespace swerve_driver
