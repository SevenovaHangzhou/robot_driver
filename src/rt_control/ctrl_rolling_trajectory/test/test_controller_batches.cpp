#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <map>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_motion_interfaces/msg/rolling_joint_target_batch.hpp"
#include "robot_rt_control_interfaces/msg/rolling_joint_control_state.hpp"
#include "rolling_test_joints.hpp"
#include "ctrl_rolling_trajectory/cubic_hermite.hpp"
#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"
#include "ctrl_rolling_trajectory/rolling_types.hpp"

namespace
{

thread_local bool track_allocations = false;
thread_local std::uint64_t tracked_allocation_count = 0U;

void countAllocation() noexcept
{
  if (track_allocations) {
    ++tracked_allocation_count;
  }
}

void * allocateUnaligned(std::size_t size) noexcept
{
  return std::malloc(size == 0U ? 1U : size);
}

void * allocateAligned(std::size_t size, std::size_t alignment) noexcept
{
  void * memory = nullptr;
  const std::size_t allocation_size = size == 0U ? 1U : size;
  return posix_memalign(&memory, alignment, allocation_size) == 0 ? memory : nullptr;
}

}  // namespace

void * operator new(std::size_t size)
{
  countAllocation();
  if (void * memory = allocateUnaligned(size)) {
    return memory;
  }
  throw std::bad_alloc();
}

void * operator new[](std::size_t size)
{
  countAllocation();
  if (void * memory = allocateUnaligned(size)) {
    return memory;
  }
  throw std::bad_alloc();
}

void operator delete(void * memory) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory) noexcept
{
  std::free(memory);
}

void operator delete(void * memory, std::size_t) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory, std::size_t) noexcept
{
  std::free(memory);
}

void * operator new(std::size_t size, const std::nothrow_t &) noexcept
{
  countAllocation();
  return allocateUnaligned(size);
}

void * operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
  countAllocation();
  return allocateUnaligned(size);
}

void operator delete(void * memory, const std::nothrow_t &) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory, const std::nothrow_t &) noexcept
{
  std::free(memory);
}

void * operator new(std::size_t size, std::align_val_t alignment)
{
  countAllocation();
  if (void * memory = allocateAligned(size, static_cast<std::size_t>(alignment))) {
    return memory;
  }
  throw std::bad_alloc();
}

void * operator new[](std::size_t size, std::align_val_t alignment)
{
  countAllocation();
  if (void * memory = allocateAligned(size, static_cast<std::size_t>(alignment))) {
    return memory;
  }
  throw std::bad_alloc();
}

void operator delete(void * memory, std::align_val_t) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory, std::align_val_t) noexcept
{
  std::free(memory);
}

void operator delete(void * memory, std::size_t, std::align_val_t) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory, std::size_t, std::align_val_t) noexcept
{
  std::free(memory);
}

void * operator new(std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  countAllocation();
  return allocateAligned(size, static_cast<std::size_t>(alignment));
}

void * operator new[](std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  countAllocation();
  return allocateAligned(size, static_cast<std::size_t>(alignment));
}

void operator delete(void * memory, std::align_val_t, const std::nothrow_t &) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory, std::align_val_t, const std::nothrow_t &) noexcept
{
  std::free(memory);
}

namespace ctrl_rolling_trajectory
{

class RollingControllerRtTestPeer
{
public:
  using Batch = robot_motion_interfaces::msg::RollingJointTargetBatch;
  using State = robot_rt_control_interfaces::msg::RollingJointControlState;

  static void submit(RollingTrajectoryController & controller, const Batch & batch)
  {
    controller.handleBatch(std::make_shared<Batch>(batch));
  }

  static State state(RollingTrajectoryController & controller)
  {
    State state;
    EXPECT_TRUE(controller.buildPublicState(state));
    return state;
  }

  static control_msgs::srv::QueryTrajectoryState::Response held(
    RollingTrajectoryController & controller)
  {
    control_msgs::srv::QueryTrajectoryState::Request request;
    control_msgs::srv::QueryTrajectoryState::Response response;
    controller.queryHeldState(request, response);
    return response;
  }

  static SessionState sessionState(const RollingTrajectoryController & controller)
  {
    return static_cast<SessionState>(controller.rt_session_state_.load(std::memory_order_acquire));
  }

  static std::uint64_t executionTime(const RollingTrajectoryController & controller)
  {
    return controller.execution_time_ns_.load(std::memory_order_acquire);
  }

  static std::string rejectionReason(const RollingTrajectoryController & controller)
  {
    return controller.admission_failure_;
  }

  static void expireAcceptedUpdate(RollingTrajectoryController & controller)
  {
    controller.rt_last_accepted_arrival_ns_ = 0U;
  }

  static void stageUnpublishedState(RollingTrajectoryController & controller)
  {
    auto & writing = controller.rt_state_exchange_.writingSlot();
    writing.session_state = SessionState::kStopping;
    writing.stop_reason = StopReason::kInternalInvariant;
  }

  static void stageCompletion(RollingTrajectoryController & controller, bool holding)
  {
    controller.stop_reason_.store(
      static_cast<std::uint8_t>(StopReason::kCompleted), std::memory_order_release);
    if (holding) {
      controller.rt_session_state_.store(
        static_cast<std::uint8_t>(SessionState::kHolding), std::memory_order_release);
    }
  }

  static void publishState(RollingTrajectoryController & controller)
  {
    controller.publishRtStateView();
  }

  static void expireState(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    auto stale = controller.readRtStateView();
    stale.published_at_ns = 0U;
    controller.rt_state_exchange_.writingSlot() = stale;
    controller.rt_state_exchange_.publish();
  }

  static void advanceBoundary(RollingTrajectoryController & controller, std::uint64_t boundary_ns)
  {
    controller.replaceable_from_ns_.store(boundary_ns, std::memory_order_release);
  }

  static void advanceExecution(RollingTrajectoryController & controller, std::uint64_t execution_ns)
  {
    controller.execution_time_ns_.store(execution_ns, std::memory_order_release);
  }

  static void consumePendingTrajectory(RollingTrajectoryController & controller)
  {
    EXPECT_EQ(
      controller.consumeLatestRtTrajectory(false),
      RollingTrajectoryController::SnapshotConsumeResult::kAccepted);
  }
};

namespace
{

using Batch = RollingControllerRtTestPeer::Batch;
using State = RollingControllerRtTestPeer::State;
using Peer = RollingControllerRtTestPeer;

constexpr std::int64_t kCycleNs = 4'000'000;

class ControllerBatchTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      actual_positions_[axis] = static_cast<double>(axis) * 0.25;
      command_positions_[axis] = actual_positions_[axis] + 0.01;
      command_handles_.emplace_back(kTestJointNames[axis], "position", &command_positions_[axis]);
      state_handles_.emplace_back(kTestJointNames[axis], "position", &actual_positions_[axis]);
    }
    feedback_age_ms_ = 12.0;
    state_handles_.emplace_back("ethercat_domain", "process_data_age_ms", &feedback_age_ms_);
    auto options = rclcpp::NodeOptions();
    options.parameter_overrides(testControllerParameters());
    controller_ = std::make_unique<RollingTrajectoryController>();
    ASSERT_EQ(
      controller_->init("test_rolling_batches", "", options),
      controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
    activate();
  }

  void TearDown() override
  {
    track_allocations = false;
    if (controller_) {
      (void)controller_->on_deactivate(rclcpp_lifecycle::State());
    }
  }

  void activate()
  {
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (auto & handle : command_handles_) {
      commands.emplace_back(handle);
    }
    for (auto & handle : state_handles_) {
      states.emplace_back(handle);
    }
    controller_->assign_interfaces(std::move(commands), std::move(states));
    ASSERT_EQ(
      controller_->on_activate(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
  }

  rclcpp::Time now() const { return controller_->get_node()->now(); }

  // Rest-to-rest move of every axis by `displacement` over `duration` from the held command.
  Batch move(
    std::uint64_t sequence, const rclcpp::Time & replace_from, double duration,
    double displacement) const
  {
    Batch batch;
    batch.epoch = Peer::state(*controller_).epoch;
    batch.sequence = sequence;
    batch.replace_from = replace_from;
    batch.points.resize(2U);
    batch.points[1].time_from_start = rclcpp::Duration::from_seconds(duration);
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      batch.points[0].positions[axis] = command_positions_[axis];
      batch.points[1].positions[axis] = command_positions_[axis] + displacement;
    }
    return batch;
  }

  controller_interface::return_type update(const rclcpp::Time & time)
  {
    return controller_->update(time, rclcpp::Duration::from_nanoseconds(kCycleNs));
  }

  // Runs `cycles` cycles from `time`, advancing it by one period each cycle.
  rclcpp::Time run(rclcpp::Time time, std::size_t cycles)
  {
    for (std::size_t cycle = 0U; cycle < cycles; ++cycle) {
      EXPECT_EQ(update(time), controller_interface::return_type::OK);
      time += rclcpp::Duration::from_nanoseconds(kCycleNs);
    }
    return time;
  }

  std::array<double, kAxisCount> command_positions_{};
  std::array<double, kAxisCount> actual_positions_{};
  double feedback_age_ms_{0.0};
  std::vector<hardware_interface::CommandInterface> command_handles_{};
  std::vector<hardware_interface::StateInterface> state_handles_{};
  std::unique_ptr<RollingTrajectoryController> controller_{};
};

TEST_F(ControllerBatchTest, ConcurrentOneKilohertzUpdatesAcceptContinuousReplacements)
{
  const auto start = now() + rclcpp::Duration::from_seconds(0.05);
  Peer::submit(*controller_, move(1U, start, 0.4, 0.0));
  auto replacement = move(2U, start + rclcpp::Duration::from_seconds(0.1), 0.4, 0.0);
  std::atomic_bool stopped{false};
  std::atomic_bool update_failed{false};
  std::thread realtime([&]() {
    auto next = std::chrono::steady_clock::now();
    while (!stopped.load()) {
      if (
        controller_->update(now(), rclcpp::Duration::from_nanoseconds(1'000'000)) !=
        controller_interface::return_type::OK) {
        update_failed.store(true);
      }
      next += std::chrono::milliseconds(1);
      std::this_thread::sleep_until(next);
    }
  });
  unsigned int rejected = 0U;
  for (unsigned int i = 0U; i < 150U; ++i) {
    replacement.sequence = i + 2U;
    replacement.replace_from = now() + rclcpp::Duration::from_seconds(0.1);
    Peer::submit(*controller_, replacement);
    rejected += Peer::state(*controller_).last_result != State::ACCEPTED ? 1U : 0U;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  stopped.store(true);
  realtime.join();
  EXPECT_FALSE(update_failed.load());
  EXPECT_EQ(rejected, 0U);
  EXPECT_EQ(Peer::state(*controller_).state, State::RUNNING);
}

TEST_F(ControllerBatchTest, HeldQueryRejectsAnExpiredSummaryUntilUpdatePublishesAgain)
{
  ASSERT_TRUE(Peer::held(*controller_).success);
  Peer::expireState(*controller_);
  const auto stale = Peer::held(*controller_);
  EXPECT_FALSE(stale.success);
  EXPECT_TRUE(stale.position.empty());
  EXPECT_EQ(update(now()), controller_interface::return_type::OK);
  const auto refreshed = Peer::held(*controller_);
  ASSERT_TRUE(refreshed.success);
  EXPECT_EQ(
    refreshed.position, std::vector<double>(command_positions_.begin(), command_positions_.end()));
}

TEST_F(ControllerBatchTest, HeldQueryDoesNotReturnThePreviousHoldAfterANewBatchIsAccepted)
{
  ASSERT_TRUE(Peer::held(*controller_).success);
  const auto start = now() + rclcpp::Duration::from_seconds(0.05);
  Peer::submit(*controller_, move(1U, start, 0.4, 0.02));
  ASSERT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);
  const auto pending = Peer::held(*controller_);
  EXPECT_FALSE(pending.success);
  EXPECT_TRUE(pending.position.empty());
}

TEST_F(ControllerBatchTest, FirstBatchHoldsUntilReplaceFromThenFollowsTheCubic)
{
  const std::array<double, kAxisCount> hold = command_positions_;
  const rclcpp::Time start = now() + rclcpp::Duration::from_seconds(0.1);
  Peer::submit(*controller_, move(1U, start, 0.4, 0.02));
  State state = Peer::state(*controller_);
  EXPECT_EQ(state.epoch, 1U);
  EXPECT_EQ(state.last_sequence, 1U);
  EXPECT_EQ(state.last_result, State::ACCEPTED);
  EXPECT_EQ(state.state, State::STARTING);

  run(start - rclcpp::Duration::from_seconds(0.05), 5U);
  EXPECT_EQ(command_positions_, hold);
  EXPECT_EQ(Peer::state(*controller_).state, State::STARTING);

  run(start, 1U);
  EXPECT_EQ(Peer::state(*controller_).state, State::RUNNING);
  run(start, 25U);
  CubicHermite segment;
  ASSERT_TRUE(buildCubicHermite(0.0, 0.0, 0.02, 0.0, 0.4, segment));
  ScalarKinematicState expected;
  ASSERT_TRUE(sampleCubicHermite(
    segment, static_cast<double>(Peer::executionTime(*controller_)) * 1.0e-9 / 0.4, expected));
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_NEAR(command_positions_[axis], hold[axis] + expected.position, 1.0e-12);
  }
}

TEST_F(ControllerBatchTest, RunningReplacementSplicesOntoTheBufferedTrajectory)
{
  const std::array<double, kAxisCount> hold = command_positions_;
  const rclcpp::Time start = now() + rclcpp::Duration::from_seconds(0.05);
  Peer::submit(*controller_, move(1U, start, 0.5, 0.1));
  rclcpp::Time time = run(start, 10U);
  ASSERT_EQ(Peer::state(*controller_).state, State::RUNNING);

  // Replace 0.3 s after the start with a slower target; the first point is the buffered cubic.
  CubicHermite original;
  ASSERT_TRUE(buildCubicHermite(0.0, 0.0, 0.1, 0.0, 0.5, original));
  ScalarKinematicState splice;
  ASSERT_TRUE(sampleCubicHermite(original, 0.3 / 0.5, splice));
  Batch replacement;
  replacement.epoch = 1U;
  replacement.sequence = 2U;
  replacement.replace_from = start + rclcpp::Duration::from_seconds(0.3);
  // A motion ends with a hold: at the end of the buffer the controller brakes (BUFFER_END),
  // so the target is followed by a stationary point to be reached exactly.
  replacement.points.resize(3U);
  replacement.points[1].time_from_start = rclcpp::Duration::from_seconds(0.3);
  replacement.points[2].time_from_start = rclcpp::Duration::from_seconds(0.32);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    replacement.points[0].positions[axis] = hold[axis] + splice.position;
    replacement.points[0].velocities[axis] = splice.velocity;
    replacement.points[1].positions[axis] = hold[axis] + 0.12;
    replacement.points[2].positions[axis] = hold[axis] + 0.12;
  }
  Peer::submit(*controller_, replacement);
  EXPECT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);

  // A first point off the buffered trajectory does not splice.
  Batch mismatch = replacement;
  mismatch.sequence = 3U;
  mismatch.points[0].positions[0] += 0.001;
  Peer::submit(*controller_, mismatch);
  EXPECT_EQ(Peer::state(*controller_).last_result, State::SPLICE_MISMATCH);

  run(time, 160U);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_NEAR(command_positions_[axis], hold[axis] + 0.12, 1.0e-12);
  }
}

TEST_F(ControllerBatchTest, BatchesBeforeTheStartExtendTheFirstOne)
{
  const std::array<double, kAxisCount> hold = command_positions_;
  const rclcpp::Time start = now() + rclcpp::Duration::from_seconds(0.2);
  Peer::submit(*controller_, move(1U, start, 0.4, 0.0));
  // Streamed before the first batch starts: splices 0.1 s into it instead of restarting.
  Batch extension = move(2U, start + rclcpp::Duration::from_seconds(0.1), 0.3, 0.04);
  Peer::submit(*controller_, extension);
  EXPECT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);
  run(start, 120U);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_NEAR(command_positions_[axis], hold[axis] + 0.04, 1.0e-3);
  }
}

TEST_F(ControllerBatchTest, RejectionsAreReportedWithTheirSequence)
{
  const rclcpp::Time start = now() + rclcpp::Duration::from_seconds(0.1);
  const auto result = [this](const Batch & batch) {
    Peer::submit(*controller_, batch);
    const State state = Peer::state(*controller_);
    EXPECT_EQ(state.last_sequence, batch.sequence);
    return state.last_result;
  };
  Batch wrong_epoch = move(1U, start, 0.4, 0.02);
  wrong_epoch.epoch = 7U;
  EXPECT_EQ(result(wrong_epoch), State::WRONG_EPOCH);
  EXPECT_EQ(result(move(2U, now(), 0.4, 0.02)), State::LATE_REPLACE);
  EXPECT_EQ(
    result(move(3U, now() + rclcpp::Duration::from_seconds(2.0), 0.4, 0.02)),
    State::HORIZON_EXCEEDED);
  Batch empty = move(4U, start, 0.4, 0.02);
  empty.points.clear();
  EXPECT_EQ(result(empty), State::INVALID_BATCH);
  Batch off_hold = move(5U, start, 0.4, 0.02);
  off_hold.points[0].positions[2] += 0.001;
  EXPECT_EQ(result(off_hold), State::SPLICE_MISMATCH);
  // Far beyond the 10 rad/s velocity limit.
  Batch beyond = move(6U, start, 0.5, 0.0);
  beyond.points[1].positions[0] = 9.0;
  EXPECT_EQ(result(beyond), State::LIMIT_EXCEEDED);
  EXPECT_EQ(result(move(6U, start, 0.4, 0.02)), State::STALE_SEQUENCE);
  EXPECT_EQ(Peer::state(*controller_).state, State::HOLDING);
  Batch final_moving = move(7U, start, 0.4, 0.02);
  final_moving.final = true;
  final_moving.points[1].velocities[3] = 0.01;
  EXPECT_EQ(result(final_moving), State::FINAL_NOT_AT_REST);
  EXPECT_EQ(result(move(8U, start, 0.4, 0.02)), State::ACCEPTED);
  EXPECT_EQ(Peer::state(*controller_).state, State::STARTING);
}

TEST_F(ControllerBatchTest, UpdateTimeoutStopsAndTheNextBatchStartsAgain)
{
  const rclcpp::Time start = now() + rclcpp::Duration::from_seconds(0.05);
  Peer::submit(*controller_, move(1U, start, 0.5, 0.05));
  rclcpp::Time time = run(start, 20U);
  ASSERT_EQ(Peer::state(*controller_).state, State::RUNNING);
  EXPECT_FALSE(Peer::held(*controller_).success);

  Peer::expireAcceptedUpdate(*controller_);
  time = run(time, 200U);
  State state = Peer::state(*controller_);
  EXPECT_EQ(state.state, State::HOLDING);
  EXPECT_EQ(state.stop_reason, State::STOP_UPDATE_TIMEOUT);
  const auto recovered = Peer::held(*controller_);
  ASSERT_TRUE(recovered.success);
  EXPECT_EQ(
    recovered.position, std::vector<double>(command_positions_.begin(), command_positions_.end()));

  // A rejected first batch starts nothing: the stop reason of the last trajectory stays.
  const rclcpp::Time restart = now() + rclcpp::Duration::from_seconds(0.05);
  Batch off_hold = move(2U, restart, 0.4, -0.02);
  off_hold.points[0].positions[0] += 0.001;
  Peer::submit(*controller_, off_hold);
  state = Peer::state(*controller_);
  EXPECT_EQ(state.last_result, State::SPLICE_MISMATCH);
  EXPECT_EQ(state.state, State::HOLDING);
  EXPECT_EQ(state.stop_reason, State::STOP_UPDATE_TIMEOUT);
  EXPECT_EQ(update(time), controller_interface::return_type::OK);
  EXPECT_TRUE(Peer::held(*controller_).success);

  // An accepted one starts a new trajectory with a cleared stop reason.
  Peer::submit(*controller_, move(3U, restart, 0.4, -0.02));
  state = Peer::state(*controller_);
  EXPECT_EQ(state.last_result, State::ACCEPTED);
  EXPECT_EQ(state.state, State::STARTING);
  EXPECT_EQ(state.stop_reason, State::STOP_NONE);
  run(restart, 2U);
  EXPECT_EQ(Peer::state(*controller_).state, State::RUNNING);
}

TEST_F(ControllerBatchTest, FinalTrajectoryRunsToItsEndWithoutFurtherUpdates)
{
  const std::array<double, kAxisCount> hold = command_positions_;
  const rclcpp::Time start = now() + rclcpp::Duration::from_seconds(0.05);
  Batch batch = move(1U, start, 0.3, 0.05);
  batch.final = true;
  Peer::submit(*controller_, batch);
  ASSERT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);
  rclcpp::Time time = run(start, 10U);
  ASSERT_EQ(Peer::state(*controller_).state, State::RUNNING);

  // No batch arrives any more; a non-final trajectory would stop on the update timeout here.
  Peer::expireAcceptedUpdate(*controller_);
  time = run(time, 80U);  // past the end at 0.3 s
  const State state = Peer::state(*controller_);
  EXPECT_EQ(state.state, State::HOLDING);
  EXPECT_EQ(state.stop_reason, State::STOP_COMPLETED);
  const auto completed = Peer::held(*controller_);
  ASSERT_TRUE(completed.success);
  EXPECT_EQ(
    completed.position, std::vector<double>(command_positions_.begin(), command_positions_.end()));
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_NEAR(command_positions_[axis], hold[axis] + 0.05, 1.0e-12);
  }
  run(time, 20U);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_NEAR(command_positions_[axis], hold[axis] + 0.05, 1.0e-12);
  }
}

TEST_F(ControllerBatchTest, NonFinalReplacementOfAFinalTrajectoryAwaitsUpdatesAgain)
{
  const std::array<double, kAxisCount> hold = command_positions_;
  const rclcpp::Time start = now() + rclcpp::Duration::from_seconds(0.05);
  Batch batch = move(1U, start, 0.5, 0.1);
  batch.final = true;
  Peer::submit(*controller_, batch);
  rclcpp::Time time = run(start, 10U);
  ASSERT_EQ(Peer::state(*controller_).state, State::RUNNING);

  CubicHermite original;
  ASSERT_TRUE(buildCubicHermite(0.0, 0.0, 0.1, 0.0, 0.5, original));
  ScalarKinematicState splice;
  ASSERT_TRUE(sampleCubicHermite(original, 0.2 / 0.5, splice));
  Batch replacement;
  replacement.epoch = 1U;
  replacement.sequence = 2U;
  replacement.replace_from = start + rclcpp::Duration::from_seconds(0.2);
  replacement.points.resize(2U);
  replacement.points[1].time_from_start = rclcpp::Duration::from_seconds(0.3);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    replacement.points[0].positions[axis] = hold[axis] + splice.position;
    replacement.points[0].velocities[axis] = splice.velocity;
    replacement.points[1].positions[axis] = hold[axis] + 0.08;
  }
  Peer::submit(*controller_, replacement);
  ASSERT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);
  time = run(time, 2U);

  Peer::expireAcceptedUpdate(*controller_);
  run(time, 200U);
  const State state = Peer::state(*controller_);
  EXPECT_EQ(state.state, State::HOLDING);
  EXPECT_EQ(state.stop_reason, State::STOP_UPDATE_TIMEOUT);
}

TEST_F(ControllerBatchTest, BufferEndWhileMovingBrakesWithinTheLimits)
{
  const rclcpp::Time start = now() + rclcpp::Duration::from_seconds(0.05);
  Batch batch = move(1U, start, 0.2, 0.05);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    batch.points[1].velocities[axis] = 0.5;
  }
  Peer::submit(*controller_, batch);
  ASSERT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);
  run(start, 150U);
  const State state = Peer::state(*controller_);
  EXPECT_EQ(state.state, State::HOLDING);
  EXPECT_EQ(state.stop_reason, State::STOP_BUFFER_END);
}

TEST_F(ControllerBatchTest, ReactivationStartsANewEpoch)
{
  const rclcpp::Time start = now() + rclcpp::Duration::from_seconds(0.1);
  Batch first_epoch = move(1U, start, 0.4, 0.02);
  ASSERT_EQ(
    controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  activate();
  EXPECT_EQ(Peer::state(*controller_).epoch, 2U);
  Peer::submit(*controller_, first_epoch);
  EXPECT_EQ(Peer::state(*controller_).last_result, State::WRONG_EPOCH);
  first_epoch.epoch = 2U;
  Peer::submit(*controller_, first_epoch);
  EXPECT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);
}

TEST_F(ControllerBatchTest, ConcurrentBatchesAndRtConsumptionRemainCoherent)
{
  const rclcpp::Time start = now() + rclcpp::Duration::from_seconds(0.05);
  Peer::submit(*controller_, move(1U, start, 0.55, 0.0));
  const auto time = run(start, 20U);
  ASSERT_EQ(Peer::state(*controller_).state, State::RUNNING);
  Batch replacement = move(2U, start + rclcpp::Duration::from_seconds(0.45), 0.1, 0.0);
  constexpr std::size_t kReplacements = 80U;
  std::atomic<std::size_t> requested{0U};
  std::atomic<std::size_t> completed{0U};
  std::atomic<std::uint64_t> update_errors{0U};
  std::thread rt_thread([&]() {
    auto next = time;
    for (std::size_t cycle = 1U; cycle <= kReplacements; ++cycle) {
      while (requested.load(std::memory_order_acquire) < cycle) {
        std::this_thread::yield();
      }
      if (update(next) != controller_interface::return_type::OK) {
        update_errors.fetch_add(1U, std::memory_order_relaxed);
      }
      next += rclcpp::Duration::from_nanoseconds(kCycleNs);
      completed.store(cycle, std::memory_order_release);
    }
  });
  // Every replacement remains ahead of the moving boundary, so every one must be accepted.
  for (std::size_t cycle = 1U; cycle <= kReplacements; ++cycle) {
    replacement.sequence = cycle + 1U;
    requested.store(cycle, std::memory_order_release);
    Peer::submit(*controller_, replacement);
    EXPECT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);
    while (completed.load(std::memory_order_acquire) < cycle) {
      std::this_thread::yield();
    }
  }
  rt_thread.join();

  ASSERT_EQ(update_errors.load(std::memory_order_relaxed), 0U);
  const State state = Peer::state(*controller_);
  EXPECT_EQ(state.state, State::RUNNING);
  EXPECT_EQ(state.stop_reason, State::STOP_NONE);
  for (const double command : command_positions_) {
    EXPECT_TRUE(std::isfinite(command));
  }
}

TEST_F(ControllerBatchTest, UnpublishedRtStateDoesNotRejectAValidRunningReplacement)
{
  const auto start = now() + rclcpp::Duration::from_seconds(0.1);
  Peer::submit(*controller_, move(1U, start, 0.5, 0.0));
  run(start, 20U);
  Peer::stageUnpublishedState(*controller_);
  Peer::submit(*controller_, move(2U, start + rclcpp::Duration::from_seconds(0.25), 0.3, 0.0));
  const auto state = Peer::state(*controller_);
  EXPECT_EQ(state.last_result, State::ACCEPTED);
  EXPECT_EQ(state.state, State::RUNNING);
  EXPECT_EQ(state.stop_reason, State::STOP_NONE);
}

TEST_F(ControllerBatchTest, PublicStateUsesACompletedPublicationForStateAndStopReason)
{
  const auto start = now() + rclcpp::Duration::from_seconds(0.1);
  Peer::submit(*controller_, move(1U, start, 0.5, 0.0));
  run(start, 20U);
  for (const bool holding : {false, true}) {
    Peer::stageCompletion(*controller_, holding);
    const auto state = Peer::state(*controller_);
    EXPECT_EQ(state.state, State::RUNNING);
    EXPECT_EQ(state.stop_reason, State::STOP_NONE);
  }
  Peer::publishState(*controller_);
  const auto completed = Peer::state(*controller_);
  EXPECT_EQ(completed.state, State::HOLDING);
  EXPECT_EQ(completed.stop_reason, State::STOP_COMPLETED);
}

TEST_F(ControllerBatchTest, StaleRtStateCannotAdmitUntilAnUpdatePublishesAgain)
{
  const auto start = now() + rclcpp::Duration::from_seconds(0.1);
  Peer::submit(*controller_, move(1U, start, 0.5, 0.0));
  const auto time = run(start, 20U);
  Peer::expireState(*controller_);
  auto replacement = move(2U, start + rclcpp::Duration::from_seconds(0.25), 0.3, 0.0);
  Peer::submit(*controller_, replacement);
  EXPECT_EQ(Peer::state(*controller_).last_result, State::NOT_ACCEPTING);
  EXPECT_EQ(Peer::rejectionReason(*controller_), "snapshot_stale");
  run(time, 1U);
  replacement.sequence = 3U;
  Peer::submit(*controller_, replacement);
  EXPECT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);
}

TEST_F(ControllerBatchTest, CommitUsesTheLiveBoundaryWhileTheNextStateIsUnpublished)
{
  const auto start = now() + rclcpp::Duration::from_seconds(0.1);
  Peer::submit(*controller_, move(1U, start, 0.5, 0.0));
  run(start, 20U);
  Peer::advanceBoundary(*controller_, 300'000'000U);
  Peer::submit(*controller_, move(2U, start + rclcpp::Duration::from_seconds(0.25), 0.3, 0.0));
  EXPECT_EQ(Peer::state(*controller_).last_result, State::LATE_REPLACE);
}

TEST_F(ControllerBatchTest, ReactivationClearsTheCompletedExecutionSummary)
{
  const auto start = now() + rclcpp::Duration::from_seconds(0.1);
  auto final = move(1U, start, 0.1, 0.0);
  final.final = true;
  Peer::submit(*controller_, final);
  run(start, 30U);
  ASSERT_EQ(Peer::state(*controller_).stop_reason, State::STOP_COMPLETED);
  ASSERT_EQ(
    controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(Peer::state(*controller_).stop_reason, State::STOP_DEACTIVATED);
  activate();
  const auto state = Peer::state(*controller_);
  EXPECT_EQ(state.state, State::HOLDING);
  EXPECT_EQ(state.stop_reason, State::STOP_NONE);
}

TEST_F(ControllerBatchTest, UnpublishedConsumptionAcknowledgesThePreviousReplacementBoundary)
{
  const auto start = now() + rclcpp::Duration::from_seconds(0.1);
  Peer::submit(*controller_, move(1U, start, 0.5, 0.0));
  run(start, 20U);  // execution 76 ms, replacement boundary 92 ms
  Peer::submit(*controller_, move(2U, start + rclcpp::Duration::from_seconds(0.092), 0.4, 0.0));
  ASSERT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);
  Peer::consumePendingTrajectory(*controller_);
  Peer::advanceBoundary(*controller_, 96'000'000U);
  Peer::submit(*controller_, move(3U, start + rclcpp::Duration::from_seconds(0.25), 0.3, 0.0));
  EXPECT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);
}

TEST_F(ControllerBatchTest, UnpublishedExecutionReservesStopTimeGrowthUntilPublication)
{
  const auto start = now() + rclcpp::Duration::from_seconds(0.1);
  Peer::submit(*controller_, move(1U, start, 0.5, 0.0));
  run(start, 20U);  // execution 76 ms
  Peer::advanceExecution(*controller_, 80'000'000U);
  Peer::advanceBoundary(*controller_, 96'000'000U);
  auto replacement = move(2U, start + rclcpp::Duration::from_seconds(0.096), 0.000001, 0.0);
  Peer::submit(*controller_, replacement);
  // The end is just beyond the 16 ms guard, but not the 4 ms of possible stop-time growth.
  EXPECT_EQ(Peer::state(*controller_).last_result, State::HORIZON_EXCEEDED);
  Peer::publishState(*controller_);
  replacement.sequence = 3U;
  Peer::submit(*controller_, replacement);
  EXPECT_EQ(Peer::state(*controller_).last_result, State::ACCEPTED);
}

TEST_F(ControllerBatchTest, UpdatePathDoesNotAllocate)
{
  const rclcpp::Time start = now() + rclcpp::Duration::from_seconds(0.05);
  Peer::submit(*controller_, move(1U, start, 0.5, 0.05));
  rclcpp::Time time = start;
  const std::uint64_t before = tracked_allocation_count;
  track_allocations = true;
  for (std::size_t cycle = 0U; cycle < 100U; ++cycle) {
    if (update(time) != controller_interface::return_type::OK) {
      track_allocations = false;
      FAIL() << "update failed at cycle " << cycle;
    }
    time += rclcpp::Duration::from_nanoseconds(kCycleNs);
  }
  track_allocations = false;
  EXPECT_EQ(tracked_allocation_count, before);
}

}  // namespace
TEST_F(ControllerBatchTest, RejectionTopicCarriesTheExactRejectedBatchAndLimit)
{
  auto reader = std::make_shared<rclcpp::Node>("rolling_rejection_test_reader");
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr report;
  auto subscription = reader->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/test_rolling_batches/rejection", rclcpp::QoS(1).reliable().transient_local(),
    [&report](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) { report = message; });
  const auto start = now() + rclcpp::Duration::from_seconds(0.1);
  const auto batch = move(1U, start, 0.4, 4.0);
  Peer::submit(*controller_, batch);
  ASSERT_EQ(Peer::state(*controller_).last_result, State::LIMIT_EXCEEDED);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!report && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(reader);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(report);
  ASSERT_EQ(report->status.size(), 1U);
  const auto & status = report->status.front();
  EXPECT_EQ(status.name, "/test_rolling_batches/rejection");
  std::map<std::string, std::string> values;
  for (const auto & item : status.values) {
    values[item.key] = item.value;
  }
  EXPECT_EQ(values["epoch"], std::to_string(batch.epoch));
  EXPECT_EQ(values["sequence"], "1");
  EXPECT_EQ(values["result"], "6");
  EXPECT_EQ(values["joint"], kTestJointNames[0]);
  EXPECT_EQ(values["quantity"], "velocity_positive");
  EXPECT_NEAR(std::stod(values["value"]), 15.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(std::stod(values["limit"]), 10.0);
  EXPECT_EQ(values["start_ns"], "0");
  EXPECT_EQ(values["end_ns"], "400000000");
}

}  // namespace ctrl_rolling_trajectory
