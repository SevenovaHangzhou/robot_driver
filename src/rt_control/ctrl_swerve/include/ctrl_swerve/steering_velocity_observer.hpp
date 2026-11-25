#ifndef CTRL_SWERVE__STEERING_VELOCITY_OBSERVER_HPP_
#define CTRL_SWERVE__STEERING_VELOCITY_OBSERVER_HPP_

#include <limits>

namespace ctrl_swerve
{
// CANopen exports position and receive age, but not its receive timestamp/counter.
// A strictly falling age proves a new frame; equal/rising ages never do. As in
// wheel_observation_broadcaster, poll at controller rate (1 kHz versus 4 ms SYNC).
// Faster/missed frames may be skipped, never invented. Receive time is estimated
// as steady update time minus age: its error is the same-cycle read-to-update delay.
class SteeringVelocityObserver final
{
public:
  void reset() noexcept;
  double observe(
    double angle, double age_ms, double now, bool valid, double timeout, double minimum,
    double maximum, double maximum_rate, double angle_tolerance) noexcept;

private:
  bool have_sample_{false};
  double poll_time_{0.0}, previous_age_{0.0}, sample_time_{0.0}, angle_{0.0};
  double rate_{std::numeric_limits<double>::quiet_NaN()};
};
}  // namespace ctrl_swerve
#endif
