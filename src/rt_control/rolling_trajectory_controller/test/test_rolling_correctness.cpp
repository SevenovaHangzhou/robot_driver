#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "rolling_trajectory_controller/rolling_buffer.hpp"

namespace rolling_trajectory_controller
{

class RollingBufferValidationTestPeer
{
public:
  static RejectCode validate(
    const RollingBuffer & buffer, const TrajectoryImage & image,
    std::size_t first_segment_index, std::size_t & validated_segment_count)
  {
    return buffer.validateCandidate(
      image, first_segment_index, validated_segment_count);
  }
};

namespace
{

constexpr std::uint64_t kMillisecondNs = 1'000'000U;

Identifier makeIdentifier(std::uint8_t seed)
{
  Identifier identifier{};
  for (std::size_t index = 0U; index < identifier.size(); ++index) {
    identifier[index] = static_cast<std::uint8_t>(seed + index);
  }
  return identifier;
}

SessionIdentity makeIdentity()
{
  return SessionIdentity{makeIdentifier(1U), makeIdentifier(21U), makeIdentifier(41U)};
}

struct OwnedPoint
{
  std::uint64_t time_ns{0U};
  std::array<double, kAxisCount> positions{};
  std::array<double, kAxisCount> velocities{};

  PointView view() const noexcept
  {
    return PointView{
      time_ns, positions.data(), positions.size(), velocities.data(), velocities.size()};
  }

  JointPoint jointPoint() const noexcept
  {
    JointPoint point;
    point.time_ns = time_ns;
    point.positions = positions;
    point.velocities = velocities;
    return point;
  }
};

OwnedPoint makePoint(std::uint64_t time_ns, double position = 0.0, double velocity = 0.0)
{
  OwnedPoint point;
  point.time_ns = time_ns;
  point.positions.fill(position);
  point.velocities.fill(velocity);
  return point;
}

DynamicEnvelope makeEnvelope()
{
  DynamicEnvelope envelope;
  envelope.source = LimitsSource::kTestOnly;
  envelope.limits_version[0] = 0xE1U;
  for (AxisEnvelope & axis : envelope.axes) {
    axis.position_lower = -100.0;
    axis.position_upper = 100.0;
    axis.velocity_positive = 100.0;
    axis.velocity_negative = 100.0;
    axis.acceleration_positive = 1.0e6;
    axis.acceleration_negative = 1.0e6;
    axis.stop_acceleration_positive = 1.0e6;
    axis.stop_acceleration_negative = 1.0e6;
    axis.position_margin_lower = 0.0;
    axis.position_margin_upper = 0.0;
  }
  return envelope;
}

BufferConfiguration makeConfiguration(std::size_t capacity = 64U)
{
  BufferConfiguration configuration;
  configuration.capacity = capacity;
  configuration.required_initial_horizon_ns = 500U * kMillisecondNs;
  configuration.max_horizon_ns = 600U * kMillisecondNs;
  configuration.splice_position_tolerance.fill(0.01);
  configuration.splice_velocity_tolerance.fill(0.03);
  return configuration;
}

template<std::size_t Size>
std::array<PointView, Size> makeViews(const std::array<OwnedPoint, Size> & points)
{
  std::array<PointView, Size> views{};
  for (std::size_t index = 0U; index < Size; ++index) {
    views[index] = points[index].view();
  }
  return views;
}

BatchView makeBatch(
  const SessionIdentity & identity, std::uint64_t sequence,
  std::uint64_t replace_from_ns, const PointView * points,
  std::size_t point_count)
{
  return BatchView{
    kProtocolMajor, kProtocolMinor, identity.controller_boot_id, identity.session_id,
    identity.client_instance_id, sequence, replace_from_ns, points, point_count};
}

void configureBuffer(RollingBuffer & buffer, std::size_t capacity = 64U)
{
  ASSERT_TRUE(buffer.configure(makeConfiguration(capacity)));
  ASSERT_TRUE(buffer.configureLimits(makeEnvelope(), true));
}

std::array<OwnedPoint, 6U> makePrime()
{
  std::array<OwnedPoint, 6U> points{};
  for (std::size_t index = 0U; index < points.size(); ++index) {
    points[index] = makePoint(index * 100U * kMillisecondNs, 0.1 * static_cast<double>(index));
  }
  return points;
}

TEST(RollingCorrectness, PrimeMustCopyTheOpenHoldExactly)
{
  RollingBuffer buffer;
  configureBuffer(buffer);
  const SessionIdentity identity = makeIdentity();
  const JointPoint anchor = makePoint(0U).jointPoint();
  ASSERT_TRUE(buffer.beginSession(identity, anchor));

  std::array<OwnedPoint, 2U> points = {
    makePoint(0U), makePoint(500U * kMillisecondNs)};
  points[0].positions[3] = 1.0e-12;
  auto views = makeViews(points);
  EXPECT_EQ(
    buffer.submit(makeBatch(identity, 1U, 0U, views.data(), views.size())),
    RejectCode::kPositionDiscontinuity);

  points[0] = makePoint(0U);
  points[0].velocities[7] = 1.0e-12;
  views = makeViews(points);
  EXPECT_EQ(
    buffer.submit(makeBatch(identity, 2U, 0U, views.data(), views.size())),
    RejectCode::kVelocityDiscontinuity);

  points[0] = makePoint(0U);
  views = makeViews(points);
  EXPECT_EQ(
    buffer.submit(makeBatch(identity, 3U, 0U, views.data(), views.size())),
    RejectCode::kNone);
}

TEST(RollingCorrectness, InteriorSpliceKeepsEverySampleBeforeRUnchanged)
{
  RollingBuffer buffer;
  configureBuffer(buffer);
  const SessionIdentity identity = makeIdentity();
  ASSERT_TRUE(buffer.beginSession(identity, makePoint(0U).jointPoint()));
  const auto prime = makePrime();
  const auto prime_views = makeViews(prime);
  ASSERT_EQ(
    buffer.submit(makeBatch(identity, 1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer.activatePending());
  const TrajectoryImage original = buffer.image();

  constexpr std::uint64_t replace_from_ns = 250U * kMillisecondNs;
  JointPoint left;
  ASSERT_TRUE(sampleTrajectoryLeftLimit(original, replace_from_ns, left));
  std::array<OwnedPoint, 4U> suffix = {
    makePoint(replace_from_ns), makePoint(350U * kMillisecondNs, 0.35),
    makePoint(450U * kMillisecondNs, 0.45), makePoint(550U * kMillisecondNs, 0.55)};
  suffix[0].positions = left.positions;
  suffix[0].velocities = left.velocities;
  suffix[0].positions[0] += 0.005;
  suffix[0].velocities[0] += 0.01;
  const auto suffix_views = makeViews(suffix);
  const AdmissionContext context{0U, 16U * kMillisecondNs, 0U};
  ASSERT_EQ(
    buffer.submit(
      makeBatch(identity, 2U, replace_from_ns, suffix_views.data(), suffix_views.size()),
      context),
    RejectCode::kNone);

  const TrajectoryImage & candidate = buffer.pendingImage();
  for (const std::uint64_t time_ns : {
      1U * kMillisecondNs, 99U * kMillisecondNs, 201U * kMillisecondNs,
      replace_from_ns - 1U})
  {
    JointPoint before;
    JointPoint after;
    ASSERT_TRUE(sampleTrajectoryImage(original, time_ns, before));
    ASSERT_TRUE(sampleTrajectoryImage(candidate, time_ns, after));
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      EXPECT_NEAR(after.positions[axis], before.positions[axis], 1.0e-12);
      EXPECT_NEAR(after.velocities[axis], before.velocities[axis], 1.0e-12);
    }
  }

  JointPoint at_boundary;
  ASSERT_TRUE(sampleTrajectoryImage(candidate, replace_from_ns, at_boundary));
  EXPECT_DOUBLE_EQ(at_boundary.positions[0], suffix[0].positions[0]);
  EXPECT_DOUBLE_EQ(at_boundary.velocities[0], suffix[0].velocities[0]);
  ASSERT_GE(candidate.point_count, 2U);
  bool found_pair = false;
  for (std::size_t index = 0U; index + 1U < candidate.point_count; ++index) {
    if (candidate.points[index].time_ns == replace_from_ns &&
      candidate.points[index + 1U].time_ns == replace_from_ns)
    {
      EXPECT_EQ(candidate.roles[index], TrajectoryPointRole::kSpliceLeft);
      EXPECT_EQ(candidate.roles[index + 1U], TrajectoryPointRole::kSpliceRight);
      found_pair = true;
    }
  }
  EXPECT_TRUE(found_pair);
}

TEST(RollingCorrectness, ReplacingTheSameBoundaryCannotAccumulateTolerance)
{
  RollingBuffer buffer;
  configureBuffer(buffer);
  const SessionIdentity identity = makeIdentity();
  ASSERT_TRUE(buffer.beginSession(identity, makePoint(0U).jointPoint()));
  const auto prime = makePrime();
  const auto prime_views = makeViews(prime);
  ASSERT_EQ(
    buffer.submit(makeBatch(identity, 1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer.activatePending());

  constexpr std::uint64_t replace_from_ns = 250U * kMillisecondNs;
  JointPoint original_left;
  ASSERT_TRUE(sampleTrajectoryLeftLimit(buffer.image(), replace_from_ns, original_left));
  std::array<OwnedPoint, 2U> first = {
    makePoint(replace_from_ns), makePoint(550U * kMillisecondNs, 0.55)};
  first[0].positions = original_left.positions;
  first[0].velocities = original_left.velocities;
  first[0].positions[0] += 0.006;
  auto views = makeViews(first);
  const AdmissionContext context{0U, 16U * kMillisecondNs, 0U};
  ASSERT_EQ(
    buffer.submit(
      makeBatch(identity, 2U, replace_from_ns, views.data(), views.size()), context),
    RejectCode::kNone);

  std::array<OwnedPoint, 2U> second = first;
  second[0].positions = original_left.positions;
  second[0].velocities = original_left.velocities;
  second[0].positions[0] += 0.012;
  views = makeViews(second);
  EXPECT_EQ(
    buffer.submit(
      makeBatch(identity, 3U, replace_from_ns, views.data(), views.size()), context),
    RejectCode::kPositionDiscontinuity);
}

TEST(RollingCorrectness, HistoryCompactionKeepsPointCountBounded)
{
  RollingBuffer buffer;
  configureBuffer(buffer, 12U);
  const SessionIdentity identity = makeIdentity();
  ASSERT_TRUE(buffer.beginSession(identity, makePoint(0U).jointPoint()));
  std::array<OwnedPoint, 6U> prime{};
  for (std::size_t index = 0U; index < prime.size(); ++index) {
    prime[index] = makePoint(index * 100U * kMillisecondNs);
  }
  auto views = makeViews(prime);
  ASSERT_EQ(
    buffer.submit(makeBatch(identity, 1U, 0U, views.data(), views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer.activatePending());

  constexpr std::uint64_t publish_period_ns = 33U * kMillisecondNs;
  constexpr std::uint64_t lead_ns = 16U * kMillisecondNs;
  for (std::uint64_t generation = 2U; generation <= 300U; ++generation) {
    const std::uint64_t replace_from_ns = (generation - 1U) * publish_period_ns;
    const std::uint64_t execution_ns = replace_from_ns - lead_ns;
    std::array<OwnedPoint, 6U> suffix{};
    for (std::size_t index = 0U; index < suffix.size(); ++index) {
      suffix[index] = makePoint(replace_from_ns + index * 100U * kMillisecondNs);
    }
    views = makeViews(suffix);
    const AdmissionContext context{execution_ns, replace_from_ns, 0U};
    ASSERT_EQ(
      buffer.submit(
        makeBatch(
          identity, generation, replace_from_ns, views.data(), views.size()),
        context),
      RejectCode::kNone) << "generation=" << generation;
    ASSERT_TRUE(buffer.activatePending());
    EXPECT_LE(buffer.image().point_count, 12U) << "generation=" << generation;
    JointPoint at_execution;
    EXPECT_TRUE(sampleTrajectoryImage(buffer.image(), execution_ns, at_execution));
  }
}

TEST(RollingCorrectness, MaxAndMinimumHorizonUseCoherentExecutionTime)
{
  const SessionIdentity identity = makeIdentity();
  RollingBuffer prime_buffer;
  configureBuffer(prime_buffer);
  ASSERT_TRUE(prime_buffer.beginSession(identity, makePoint(0U).jointPoint()));
  std::array<OwnedPoint, 2U> prime = {
    makePoint(0U), makePoint(600U * kMillisecondNs + 1U)};
  auto prime_views = makeViews(prime);
  EXPECT_EQ(
    prime_buffer.submit(
      makeBatch(identity, 1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kHorizonExceeded);

  prime[1] = makePoint(600U * kMillisecondNs);
  prime_views = makeViews(prime);
  ASSERT_EQ(
    prime_buffer.submit(
      makeBatch(identity, 2U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(prime_buffer.activatePending());

  constexpr std::uint64_t execution_ns = 100U * kMillisecondNs;
  constexpr std::uint64_t replace_from_ns = 116U * kMillisecondNs;
  JointPoint left;
  ASSERT_TRUE(sampleTrajectoryLeftLimit(
    prime_buffer.image(), replace_from_ns, left));
  std::array<OwnedPoint, 2U> suffix = {
    makePoint(replace_from_ns), makePoint(701U * kMillisecondNs)};
  suffix[0].positions = left.positions;
  suffix[0].velocities = left.velocities;
  auto suffix_views = makeViews(suffix);
  EXPECT_EQ(
    prime_buffer.submit(
      makeBatch(
        identity, 3U, replace_from_ns, suffix_views.data(), suffix_views.size()),
      AdmissionContext{execution_ns, replace_from_ns, 0U}),
    RejectCode::kHorizonExceeded);

  suffix[1] = makePoint(300U * kMillisecondNs);
  suffix_views = makeViews(suffix);
  EXPECT_EQ(
    prime_buffer.submit(
      makeBatch(
        identity, 4U, replace_from_ns, suffix_views.data(), suffix_views.size()),
      AdmissionContext{execution_ns, replace_from_ns, 200U * kMillisecondNs}),
    RejectCode::kInsufficientHorizon);
}

TEST(RollingCorrectness, IncrementalValidationChecksOnlyTheChangedSuffix)
{
  RollingBuffer buffer;
  configureBuffer(buffer);
  const SessionIdentity identity = makeIdentity();
  ASSERT_TRUE(buffer.beginSession(identity, makePoint(0U).jointPoint()));
  const auto prime = makePrime();
  const auto prime_views = makeViews(prime);
  ASSERT_EQ(
    buffer.submit(makeBatch(identity, 1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer.activatePending());

  constexpr std::uint64_t replace_from_ns = 400U * kMillisecondNs;
  JointPoint left;
  ASSERT_TRUE(sampleTrajectoryLeftLimit(buffer.image(), replace_from_ns, left));
  std::array<OwnedPoint, 2U> suffix = {
    makePoint(replace_from_ns), makePoint(550U * kMillisecondNs, 0.55)};
  suffix[0].positions = left.positions;
  suffix[0].velocities = left.velocities;
  const auto suffix_views = makeViews(suffix);

  PreparedSubmission prepared;
  ASSERT_EQ(
    buffer.prepare(
      makeBatch(
        identity, 2U, replace_from_ns, suffix_views.data(), suffix_views.size()),
      AdmissionContext{0U, 16U * kMillisecondNs, 0U}, prepared),
    RejectCode::kNone);
  EXPECT_EQ(prepared.validated_segment_count, suffix.size() - 1U);
  EXPECT_EQ(prepared.first_validated_segment_index, 5U);

  std::size_t full_count = 0U;
  EXPECT_EQ(
    RollingBufferValidationTestPeer::validate(
      buffer, prepared.candidate, 0U, full_count),
    RejectCode::kNone);
  EXPECT_GT(full_count, prepared.validated_segment_count);

  TrajectoryImage invalid = prepared.candidate;
  invalid.points[invalid.point_count - 1U].positions[0] = 1'000.0;
  std::size_t incremental_invalid_count = 0U;
  std::size_t full_invalid_count = 0U;
  const RejectCode incremental = RollingBufferValidationTestPeer::validate(
    buffer, invalid, prepared.first_validated_segment_index,
    incremental_invalid_count);
  const RejectCode full = RollingBufferValidationTestPeer::validate(
    buffer, invalid, 0U, full_invalid_count);
  EXPECT_EQ(incremental, RejectCode::kPositionLimit);
  EXPECT_EQ(full, incremental);
  EXPECT_LT(incremental_invalid_count, full_invalid_count);
}

TEST(RollingCorrectness, MonotonicCursorMatchesReferenceWithConstantSearchWork)
{
  TrajectoryImage image;
  image.generation = 7U;
  image.point_count = 64U;
  for (std::size_t index = 0U; index < image.point_count; ++index) {
    image.points[index] = makePoint(
      index * 100U * kMillisecondNs, 0.01 * static_cast<double>(index)).jointPoint();
  }
  ASSERT_TRUE(trajectoryImageStructureIsValid(image));

  MonotonicTrajectoryCursor cursor;
  std::size_t maximum_advances_in_one_sample = 0U;
  const std::uint64_t end_time_ns = image.points[image.point_count - 1U].time_ns;
  for (std::uint64_t time_ns = 0U; time_ns <= end_time_ns;
    time_ns += 4U * kMillisecondNs)
  {
    JointPoint reference;
    JointPoint monotonic;
    std::size_t advances = 0U;
    ASSERT_TRUE(sampleTrajectoryImage(image, time_ns, reference));
    ASSERT_TRUE(sampleTrajectoryImageMonotonic(image, time_ns, cursor, monotonic, &advances));
    maximum_advances_in_one_sample = std::max(maximum_advances_in_one_sample, advances);
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      EXPECT_DOUBLE_EQ(monotonic.positions[axis], reference.positions[axis]);
      EXPECT_DOUBLE_EQ(monotonic.velocities[axis], reference.velocities[axis]);
    }
  }
  EXPECT_LE(maximum_advances_in_one_sample, 1U);
}

TEST(RollingCorrectness, MonotonicCursorHandlesSpliceGenerationAndTimeReset)
{
  TrajectoryImage image;
  image.generation = 11U;
  image.point_count = 4U;
  image.points[0] = makePoint(0U, 0.0).jointPoint();
  image.points[1] = makePoint(100U * kMillisecondNs, 0.1).jointPoint();
  image.points[2] = makePoint(100U * kMillisecondNs, 0.105).jointPoint();
  image.points[3] = makePoint(200U * kMillisecondNs, 0.2).jointPoint();
  image.roles[1] = TrajectoryPointRole::kSpliceLeft;
  image.roles[2] = TrajectoryPointRole::kSpliceRight;
  ASSERT_TRUE(trajectoryImageStructureIsValid(image));

  MonotonicTrajectoryCursor cursor;
  for (const std::uint64_t time_ns : {
      96U * kMillisecondNs, 100U * kMillisecondNs, 104U * kMillisecondNs})
  {
    JointPoint reference;
    JointPoint monotonic;
    ASSERT_TRUE(sampleTrajectoryImage(image, time_ns, reference));
    ASSERT_TRUE(sampleTrajectoryImageMonotonic(image, time_ns, cursor, monotonic));
    EXPECT_EQ(monotonic.positions, reference.positions);
    EXPECT_EQ(monotonic.velocities, reference.velocities);
  }

  JointPoint at_splice;
  ASSERT_TRUE(sampleTrajectoryImageMonotonic(
    image, 100U * kMillisecondNs, cursor, at_splice));
  EXPECT_DOUBLE_EQ(at_splice.positions[0], 0.105);

  TrajectoryImage next_generation = image;
  next_generation.generation = 12U;
  next_generation.point_count = 2U;
  next_generation.points[0] = image.points[2];
  next_generation.roles[0] = TrajectoryPointRole::kNormal;
  next_generation.points[1] = image.points[3];
  next_generation.roles[1] = TrajectoryPointRole::kNormal;
  ASSERT_TRUE(trajectoryImageStructureIsValid(next_generation));

  JointPoint reference;
  JointPoint monotonic;
  ASSERT_TRUE(sampleTrajectoryImage(next_generation, 104U * kMillisecondNs, reference));
  ASSERT_TRUE(sampleTrajectoryImageMonotonic(
    next_generation, 104U * kMillisecondNs, cursor, monotonic));
  EXPECT_EQ(monotonic.positions, reference.positions);
  EXPECT_EQ(monotonic.velocities, reference.velocities);
}

}  // namespace
}  // namespace rolling_trajectory_controller
