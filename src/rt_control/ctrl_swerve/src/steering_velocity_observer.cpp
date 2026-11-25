#include "ctrl_swerve/steering_velocity_observer.hpp"

#include <cmath>

namespace ctrl_swerve
{
void SteeringVelocityObserver::reset() noexcept
{
  have_sample_ = false;
  poll_time_ = previous_age_ = sample_time_ = angle_ = 0.0;
  rate_ = std::numeric_limits<double>::quiet_NaN();
}

double SteeringVelocityObserver::observe(
  double angle, double age_ms, double now, bool valid, double timeout, double minimum,
  double maximum, double maximum_rate, double angle_tolerance) noexcept
{
  if (
    !valid || !std::isfinite(angle) || !std::isfinite(age_ms) || !std::isfinite(now) ||
    now <= 0.0 || age_ms < 0.0 || age_ms * 0.001 > timeout || angle < minimum || angle > maximum) {
    reset();
    return rate_;
  }
  const double received = now - age_ms * 0.001;
  if (have_sample_ && (now <= poll_time_ || now - poll_time_ > timeout)) {
    reset();
  }
  if (!have_sample_) {
    have_sample_ = true;
    poll_time_ = now;
    previous_age_ = age_ms;
    sample_time_ = received;
    angle_ = angle;
    return rate_;
  }
  const bool new_frame = age_ms < previous_age_;
  previous_age_ = age_ms;
  poll_time_ = now;
  if (new_frame) {
    const double interval = received - sample_time_;
    // Mechanical-travel angles are bounded, not circular. Never wrap an encoder
    // reset/jump into a small apparent displacement or bridge a stale interval.
    if (
      interval <= 0.0 || interval > timeout ||
      std::abs(angle - angle_) > maximum_rate * interval + angle_tolerance) {
      reset();
      return rate_;
    }
    rate_ = (angle - angle_) / interval;
    sample_time_ = received;
    angle_ = angle;
  } else if (now - sample_time_ > timeout) {
    reset();
    return rate_;
  }
  return rate_;
}
}  // namespace ctrl_swerve
