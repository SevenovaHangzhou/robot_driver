#ifndef CTRL_ROLLING_TRAJECTORY_TEST__ROLLING_BUFFER_FIXTURE_HPP_
#define CTRL_ROLLING_TRAJECTORY_TEST__ROLLING_BUFFER_FIXTURE_HPP_

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "ctrl_rolling_trajectory/rolling_buffer.hpp"

namespace ctrl_rolling_trajectory::test
{

inline constexpr std::uint64_t kMs = 1'000'000U;

inline JointPoint point(std::uint64_t ms, double position = 0.0, double velocity = 0.0)
{
  JointPoint result;
  result.time_ns = ms * kMs;
  result.positions.fill(position);
  result.velocities.fill(velocity);
  return result;
}

inline std::vector<PointView> views(const std::vector<JointPoint> & points)
{
  std::vector<PointView> result;
  for (const auto & p : points) {
    result.push_back(
      {p.time_ns, p.positions.data(), p.positions.size(), p.velocities.data(),
       p.velocities.size()});
  }
  return result;
}

class BufferFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config.capacity = 64U;
    config.max_horizon_ns = 2'000U * kMs;
    config.splice_position_tolerance.fill(0.01);
    config.splice_velocity_tolerance.fill(0.03);
    for (auto & axis : envelope.axes) {
      axis = AxisEnvelope{-100.0, 100.0, 100.0, 100.0, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 0.0, 0.0};
    }
    configure();
  }

  void configure()
  {
    ASSERT_TRUE(buffer.configure(config, envelope));
    ASSERT_TRUE(buffer.reset(JointPoint{}));
  }

  RejectCode prepare(
    const std::vector<JointPoint> & points, const AdmissionContext & context,
    TrajectoryImage & candidate)
  {
    const auto input = views(points);
    return buffer.prepare(
      {points.empty() ? 0U : points[0].time_ns, input.data(), input.size()}, context, candidate);
  }

  RejectCode admit(const std::vector<JointPoint> & points, AdmissionContext context = {})
  {
    TrajectoryImage candidate;
    auto result = prepare(points, context, candidate);
    return result == RejectCode::kNone ? buffer.commit(candidate, context) : result;
  }

  RollingBuffer buffer;
  BufferConfiguration config;
  DynamicEnvelope envelope;
};

}  // namespace ctrl_rolling_trajectory::test

#endif  // CTRL_ROLLING_TRAJECTORY_TEST__ROLLING_BUFFER_FIXTURE_HPP_
