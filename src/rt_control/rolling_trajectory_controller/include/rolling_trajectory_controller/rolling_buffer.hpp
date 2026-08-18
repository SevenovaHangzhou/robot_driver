#ifndef ROLLING_TRAJECTORY_CONTROLLER__ROLLING_BUFFER_HPP_
#define ROLLING_TRAJECTORY_CONTROLLER__ROLLING_BUFFER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

#include "rolling_trajectory_controller/limit_checker.hpp"
#include "rolling_trajectory_controller/rolling_types.hpp"

namespace rolling_trajectory_controller
{

class RollingBufferValidationTestPeer;

bool sampleTrajectoryImage(
  const TrajectoryImage & image, std::uint64_t time_ns, JointPoint & point) noexcept;
bool sampleTrajectoryLeftLimit(
  const TrajectoryImage & image, std::uint64_t time_ns, JointPoint & point) noexcept;
bool trajectoryImageStructureIsValid(const TrajectoryImage & image) noexcept;

struct MonotonicTrajectoryCursor
{
  std::size_t point_index{0U};
  std::uint64_t generation{0U};
  std::uint64_t last_time_ns{0U};
  bool initialized{false};

  void reset() noexcept
  {
    *this = MonotonicTrajectoryCursor{};
  }
};

bool sampleTrajectoryImageMonotonic(
  const TrajectoryImage & image, std::uint64_t time_ns,
  MonotonicTrajectoryCursor & cursor, JointPoint & point,
  std::size_t * segment_advances = nullptr) noexcept;

struct BufferConfiguration
{
  std::size_t capacity{0U};
  std::uint64_t required_initial_horizon_ns{0U};
  std::uint64_t max_horizon_ns{0U};
  std::array<double, kAxisCount> splice_position_tolerance{};
  std::array<double, kAxisCount> splice_velocity_tolerance{};
};

struct AdmissionContext
{
  std::uint64_t execution_time_ns{0U};
  std::uint64_t replaceable_from_ns{0U};
  std::uint64_t minimum_horizon_ns{0U};
};

struct PreparedSubmission
{
  TrajectoryImage candidate{};
  SessionIdentity identity{};
  std::uint64_t validation_base_generation{0U};
  std::uint64_t sequence{0U};
  std::size_t first_validated_segment_index{0U};
  std::size_t validated_segment_count{0U};
  bool valid{false};
};

class RollingBuffer
{
public:
  bool configure(std::size_t capacity) noexcept;
  bool configure(const BufferConfiguration & configuration) noexcept;
  bool configureLimits(
    const DynamicEnvelope & envelope, bool allow_test_only_limits) noexcept;

  [[nodiscard]] bool configured() const noexcept;
  [[nodiscard]] std::size_t capacity() const noexcept;
  [[nodiscard]] const TrajectoryImage & image() const noexcept;
  [[nodiscard]] const TrajectoryImage & pendingImage() const noexcept;
  [[nodiscard]] bool hasPending() const noexcept;
  [[nodiscard]] SessionState sessionState() const noexcept;
  [[nodiscard]] std::uint64_t lastSeenSequence() const noexcept;
  [[nodiscard]] std::uint64_t lastAcceptedSequence() const noexcept;
  [[nodiscard]] std::uint64_t validationBaseGeneration() const noexcept;
  [[nodiscard]] const SessionIdentity & identity() const noexcept;

  RejectCode replace(const PointView * points, std::size_t point_count) noexcept;
  bool beginSession(
    const SessionIdentity & identity, const JointPoint & prime_anchor) noexcept;
  RejectCode prepare(
    const BatchView & batch, const AdmissionContext & context,
    PreparedSubmission & submission) noexcept;
  RejectCode commit(
    PreparedSubmission & submission, const AdmissionContext & context) noexcept;
  RejectCode submit(const BatchView & batch) noexcept;
  RejectCode submit(
    const BatchView & batch, const AdmissionContext & context) noexcept;
  bool activatePending() noexcept;
  bool acknowledgeActiveGeneration(std::uint64_t generation) noexcept;
  bool requestStop() noexcept;
  bool markHolding() noexcept;
  bool finishSession() noexcept;
  void terminateSession() noexcept;
  bool resetTerminated() noexcept;
  bool sampleValidationHead(std::uint64_t time_ns, JointPoint & point) const noexcept;

private:
  friend class RollingBufferValidationTestPeer;

  RejectCode validateInput(const PointView * points, std::size_t point_count) const noexcept;
  void copyPoint(const PointView & input, JointPoint & output) const noexcept;
  RejectCode validateCandidate(
    const TrajectoryImage & candidate, std::size_t first_segment_index,
    std::size_t & validated_segment_count) const noexcept;
  RejectCode validateHorizon(
    const TrajectoryImage & candidate, const AdmissionContext & context,
    bool priming) const noexcept;
  const TrajectoryImage & validationHead() const noexcept;
  void clearSessionData() noexcept;

  bool configured_{false};
  BufferConfiguration configuration_{};
  TrajectoryImage image_{};
  TrajectoryImage pending_image_{};
  bool pending_valid_{false};
  SessionIdentity identity_{};
  JointPoint prime_anchor_{};
  SessionState session_state_{SessionState::kNone};
  std::uint64_t last_seen_sequence_{0U};
  std::uint64_t last_accepted_sequence_{0U};
  LimitChecker limit_checker_{};
};

}  // namespace rolling_trajectory_controller

#endif  // ROLLING_TRAJECTORY_CONTROLLER__ROLLING_BUFFER_HPP_
