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
#include "robot_rt_control_interfaces/srv/close_rolling_joint_session.hpp"
#include "robot_rt_control_interfaces/srv/open_rolling_joint_session.hpp"
#include "rolling_trajectory_controller/cubic_hermite.hpp"
#include "rolling_trajectory_controller/rolling_trajectory_controller.hpp"
#include "rolling_trajectory_controller/rolling_types.hpp"

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

void * operator new(
  std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  countAllocation();
  return allocateAligned(size, static_cast<std::size_t>(alignment));
}

void * operator new[](
  std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  countAllocation();
  return allocateAligned(size, static_cast<std::size_t>(alignment));
}

void operator delete(
  void * memory, std::align_val_t, const std::nothrow_t &) noexcept
{
  std::free(memory);
}

void operator delete[](
  void * memory, std::align_val_t, const std::nothrow_t &) noexcept
{
  std::free(memory);
}

namespace rolling_trajectory_controller
{

class RollingControllerRtTestPeer
{
public:
  using Batch = robot_motion_interfaces::msg::RollingJointTargetBatch;
  using Close = robot_rt_control_interfaces::srv::CloseRollingJointSession;
  using Open = robot_rt_control_interfaces::srv::OpenRollingJointSession;

  static Identifier bootId(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    return controller.controller_boot_id_;
  }

  static Open::Response open(
    RollingTrajectoryController & controller, const Open::Request & request)
  {
    auto request_pointer = std::make_shared<Open::Request>(request);
    auto response = std::make_shared<Open::Response>();
    controller.handleOpen(request_pointer, response);
    return *response;
  }

  static Close::Response close(
    RollingTrajectoryController & controller, const Close::Request & request)
  {
    auto request_pointer = std::make_shared<Close::Request>(request);
    auto response = std::make_shared<Close::Response>();
    controller.handleClose(request_pointer, response);
    return *response;
  }

  static void submit(RollingTrajectoryController & controller, const Batch & batch)
  {
    controller.handleUpdate(std::make_shared<Batch>(batch));
  }

  static SessionState sessionState(const RollingTrajectoryController & controller)
  {
    return static_cast<SessionState>(
      controller.rt_session_state_.load(std::memory_order_acquire));
  }

  static StopReason stopReason(const RollingTrajectoryController & controller)
  {
    return static_cast<StopReason>(controller.stop_reason_.load(std::memory_order_acquire));
  }

  static std::uint64_t executionTime(const RollingTrajectoryController & controller)
  {
    return controller.execution_time_ns_.load(std::memory_order_acquire);
  }

  static std::uint64_t activeGeneration(const RollingTrajectoryController & controller)
  {
    return controller.active_generation_.load(std::memory_order_acquire);
  }

  static std::uint64_t replaceableFrom(const RollingTrajectoryController & controller)
  {
    return controller.replaceable_from_ns_.load(std::memory_order_acquire);
  }

  static RejectCode lastReject(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    return controller.last_reject_code_;
  }

  static std::uint8_t invariantStage(const RollingTrajectoryController & controller)
  {
    return controller.rt_invariant_stage_.load(std::memory_order_acquire);
  }

  static void expirePrime(RollingTrajectoryController & controller)
  {
    controller.prime_start_time_ns_.store(0U, std::memory_order_release);
  }

  static void expireAcceptedUpdate(RollingTrajectoryController & controller)
  {
    controller.rt_last_accepted_arrival_ns_ = 0U;
  }

  static std::uint64_t acceptedCount(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    return controller.accepted_count_;
  }

  static std::uint64_t rejectedCount(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    return controller.rejected_count_;
  }

  static std::uint64_t invariantFailureCount(
    const RollingTrajectoryController & controller)
  {
    return controller.invariant_failure_count_.load(std::memory_order_acquire);
  }
};

namespace
{

using Batch = robot_motion_interfaces::msg::RollingJointTargetBatch;
using Close = robot_rt_control_interfaces::srv::CloseRollingJointSession;
using Open = robot_rt_control_interfaces::srv::OpenRollingJointSession;
using PublicServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
using PublicSessionState = robot_rt_control_interfaces::msg::RollingSessionState;

constexpr std::uint64_t kCycleNs = 4'000'000U;
constexpr std::array<std::uint8_t, 32> kAxisSetHash = {
  0x25U, 0xc6U, 0xe8U, 0x2bU, 0xf5U, 0x05U, 0xcaU, 0x9eU,
  0xb9U, 0x9dU, 0xb1U, 0xc6U, 0x45U, 0xabU, 0x75U, 0xd7U,
  0xecU, 0xdeU, 0x01U, 0x53U, 0xfaU, 0xafU, 0x6aU, 0x74U,
  0x92U, 0xc6U, 0x21U, 0x0cU, 0x4dU, 0x36U, 0x25U, 0x26U};

unique_identifier_msgs::msg::UUID makeUuid(std::uint8_t seed)
{
  unique_identifier_msgs::msg::UUID id;
  for (std::size_t index = 0U; index < id.uuid.size(); ++index) {
    id.uuid[index] = static_cast<std::uint8_t>(seed + index);
  }
  return id;
}

Open::Request makeOpenRequest(const Identifier & boot_id, std::uint8_t request_seed = 20U)
{
  Open::Request request;
  request.protocol_major = Open::Request::PROTOCOL_MAJOR;
  request.protocol_minor = Open::Request::PROTOCOL_MINOR;
  request.request_id = makeUuid(request_seed);
  request.expected_controller_boot_id.uuid = boot_id;
  request.client_instance_id = makeUuid(40U);
  request.axis_set_hash = kAxisSetHash;
  return request;
}

Close::Request makeCloseRequest(
  const Open::Response & open, std::uint8_t request_seed,
  std::uint8_t operation = Close::Request::REQUEST_STOP)
{
  Close::Request request;
  request.protocol_major = Close::Request::PROTOCOL_MAJOR;
  request.protocol_minor = Close::Request::PROTOCOL_MINOR;
  request.request_id = makeUuid(request_seed);
  request.controller_boot_id = open.controller_boot_id;
  request.session_id = open.session_id;
  request.client_instance_id = open.client_instance_id;
  request.operation = operation;
  return request;
}

Batch makeTrajectory(
  const Open::Response & open, std::uint64_t duration_ns, double displacement)
{
  Batch batch;
  batch.protocol_major = Batch::PROTOCOL_MAJOR;
  batch.protocol_minor = Batch::PROTOCOL_MINOR;
  batch.controller_boot_id = open.controller_boot_id;
  batch.session_id = open.session_id;
  batch.client_instance_id = open.client_instance_id;
  batch.sequence = 1U;
  batch.replace_from_ns = 0U;
  batch.points.resize(2U);
  batch.points[0].time_from_session_start_ns = 0U;
  batch.points[1].time_from_session_start_ns = duration_ns;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    batch.points[0].positions[axis] = open.hold_positions[axis];
    batch.points[1].positions[axis] = open.hold_positions[axis] + displacement;
    batch.points[0].velocities[axis] = 0.0;
    batch.points[1].velocities[axis] = 0.0;
  }
  return batch;
}

Batch makeReplacement(
  const Open::Response & open, std::uint64_t sequence,
  std::uint64_t replace_from_ns, std::uint64_t duration_ns,
  double original_displacement, double replacement_displacement)
{
  CubicHermite original;
  EXPECT_TRUE(
    buildCubicHermite(
      0.0, 0.0, original_displacement, 0.0,
      static_cast<double>(duration_ns) * 1.0e-9, original));
  ScalarKinematicState splice;
  EXPECT_TRUE(
    sampleCubicHermite(
      original,
      static_cast<double>(replace_from_ns) / static_cast<double>(duration_ns),
      splice));

  Batch batch;
  batch.protocol_major = Batch::PROTOCOL_MAJOR;
  batch.protocol_minor = Batch::PROTOCOL_MINOR;
  batch.controller_boot_id = open.controller_boot_id;
  batch.session_id = open.session_id;
  batch.client_instance_id = open.client_instance_id;
  batch.sequence = sequence;
  batch.replace_from_ns = replace_from_ns;
  batch.points.resize(2U);
  batch.points[0].time_from_session_start_ns = replace_from_ns;
  batch.points[1].time_from_session_start_ns = duration_ns;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    batch.points[0].positions[axis] = open.hold_positions[axis] + splice.position;
    batch.points[0].velocities[axis] = splice.velocity;
    batch.points[1].positions[axis] =
      open.hold_positions[axis] + replacement_displacement;
    batch.points[1].velocities[axis] = 0.0;
  }
  return batch;
}

Batch makeHoldWindow(
  const Open::Response & open, std::uint64_t sequence,
  std::uint64_t replace_from_ns)
{
  constexpr std::size_t kPointCount = 6U;
  constexpr std::uint64_t kKnotIntervalNs = 100'000'000U;
  Batch batch;
  batch.protocol_major = Batch::PROTOCOL_MAJOR;
  batch.protocol_minor = Batch::PROTOCOL_MINOR;
  batch.controller_boot_id = open.controller_boot_id;
  batch.session_id = open.session_id;
  batch.client_instance_id = open.client_instance_id;
  batch.sequence = sequence;
  batch.replace_from_ns = replace_from_ns;
  batch.points.resize(kPointCount);
  for (std::size_t point_index = 0U; point_index < kPointCount; ++point_index) {
    auto & point = batch.points[point_index];
    point.time_from_session_start_ns =
      replace_from_ns + static_cast<std::uint64_t>(point_index) * kKnotIntervalNs;
    point.positions = open.hold_positions;
    point.velocities.fill(0.0);
  }
  return batch;
}

std::uint64_t percentileNanoseconds(
  std::vector<std::uint64_t> samples, double percentile)
{
  if (samples.empty()) {
    return 0U;
  }
  std::sort(samples.begin(), samples.end());
  const double scaled_index =
    percentile * static_cast<double>(samples.size() - 1U);
  const std::size_t index = static_cast<std::size_t>(std::ceil(scaled_index));
  return samples[index];
}

class RtUpdateTest : public ::testing::Test
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
      command_handles_.emplace_back(kJointNames[axis], "position", &command_positions_[axis]);
      state_handles_.emplace_back(kJointNames[axis], "position", &actual_positions_[axis]);
    }
    feedback_age_ms_ = 12.0;
    state_handles_.emplace_back(
      "ethercat_domain", "process_data_age_ms", &feedback_age_ms_);

    auto options = rclcpp::NodeOptions();
    options.parameter_overrides(
        {
          rclcpp::Parameter("configuration_source", "test_only"),
          rclcpp::Parameter("allow_test_only_configuration", true),
          rclcpp::Parameter(
            "splice_position_tolerances", std::vector<double>(kAxisCount, 1.0e-9)),
          rclcpp::Parameter(
            "splice_velocity_tolerances", std::vector<double>(kAxisCount, 1.0e-9)),
          rclcpp::Parameter(
            "takeover_tolerances", std::vector<double>(kAxisCount, 0.125))});
    controller_ = std::make_unique<RollingTrajectoryController>();
    ASSERT_EQ(
      controller_->init("test_rt_update_controller", "", options),
      controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);

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

  void TearDown() override
  {
    track_allocations = false;
    if (controller_) {
      (void)controller_->on_deactivate(rclcpp_lifecycle::State());
    }
  }

  Open::Response open(std::uint8_t request_seed = 20U)
  {
    return RollingControllerRtTestPeer::open(
      *controller_,
      makeOpenRequest(RollingControllerRtTestPeer::bootId(*controller_), request_seed));
  }

  controller_interface::return_type update(
    std::int64_t period_ns =
    static_cast<std::int64_t>(kCycleNs))
  {
    return controller_->update(
      rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(period_ns));
  }

  std::array<double, kAxisCount> command_positions_{};
  std::array<double, kAxisCount> actual_positions_{};
  double feedback_age_ms_{0.0};
  std::vector<hardware_interface::CommandInterface> command_handles_{};
  std::vector<hardware_interface::StateInterface> state_handles_{};
  std::unique_ptr<RollingTrajectoryController> controller_{};
};

TEST_F(RtUpdateTest, PrimeActivatesAtTimeZeroThenSamplesTheCubic)
{
  const std::array<double, kAxisCount> initial_commands = command_positions_;
  const Open::Response response = open();
  ASSERT_TRUE(response.accepted);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 500'000'000U, 0.02));

  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning)
    << "invariant_stage=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::invariantStage(*controller_));
  EXPECT_EQ(RollingControllerRtTestPeer::activeGeneration(*controller_), 1U);
  EXPECT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), 0U);
  EXPECT_EQ(command_positions_, initial_commands);

  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), kCycleNs);
  CubicHermite segment;
  ASSERT_TRUE(buildCubicHermite(0.0, 0.0, 0.02, 0.0, 0.5, segment));
  ScalarKinematicState expected;
  ASSERT_TRUE(sampleCubicHermite(segment, 0.008, expected));
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_NEAR(
      command_positions_[axis], initial_commands[axis] + expected.position,
      1.0e-15);
    EXPECT_TRUE(std::isfinite(command_positions_[axis]));
  }
}

TEST_F(RtUpdateTest, RunningSuffixReplacementSwitchesGenerationWithoutRestartingTime)
{
  constexpr std::uint64_t duration_ns = 500'000'000U;
  constexpr std::uint64_t replace_from_ns = 20'000'000U;
  constexpr double original_displacement = 0.1;
  constexpr double replacement_displacement = 0.08;
  const Open::Response response = open();
  ASSERT_TRUE(response.accepted);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, duration_ns, original_displacement));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), kCycleNs);

  RollingControllerRtTestPeer::submit(
    *controller_,
    makeReplacement(
      response, 2U, replace_from_ns, duration_ns,
      original_displacement, replacement_displacement));
  EXPECT_EQ(RollingControllerRtTestPeer::activeGeneration(*controller_), 1U);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(RollingControllerRtTestPeer::activeGeneration(*controller_), 2U);
  EXPECT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), 2U * kCycleNs);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning)
    << "stop_reason=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::stopReason(*controller_)) <<
    ", invariant_stage=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::invariantStage(*controller_));
  EXPECT_EQ(RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kNone);

  for (std::size_t cycle = 0U; cycle < 4U; ++cycle) {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  ASSERT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), 24'000'000U);

  CubicHermite original;
  ASSERT_TRUE(
    buildCubicHermite(
      0.0, 0.0, original_displacement, 0.0, 0.5, original));
  ScalarKinematicState splice;
  ASSERT_TRUE(sampleCubicHermite(original, 0.04, splice));
  CubicHermite replacement;
  ASSERT_TRUE(
    buildCubicHermite(
      splice.position, splice.velocity, replacement_displacement, 0.0,
      0.48, replacement));
  ScalarKinematicState expected;
  ASSERT_TRUE(sampleCubicHermite(replacement, 4.0 / 480.0, expected));
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_NEAR(
      command_positions_[axis], response.hold_positions[axis] + expected.position,
      1.0e-15);
    EXPECT_TRUE(std::isfinite(command_positions_[axis]));
  }
}

TEST_F(RtUpdateTest, CloseBeforePrimeStopsAtHoldAndCanBeFinalized)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.accepted);
  const Close::Response stopping = RollingControllerRtTestPeer::close(
    *controller_, makeCloseRequest(response, 60U));
  ASSERT_TRUE(stopping.accepted);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kHolding);
  EXPECT_EQ(
    RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kGracefulClose);

  const Close::Response finalized = RollingControllerRtTestPeer::close(
    *controller_, makeCloseRequest(response, 61U, Close::Request::FINALIZE));
  EXPECT_TRUE(finalized.accepted);
  EXPECT_TRUE(finalized.completed);
}

TEST_F(RtUpdateTest, LowWaterEqualityStopsBeforeTheShortBufferCanExhaust)
{
  constexpr std::uint64_t duration_ns = 500'000'000U;
  constexpr std::uint64_t scheduling_guard_ns = 16'000'000U;
  constexpr std::uint64_t low_water_boundary_ns = duration_ns - scheduling_guard_ns;
  const Open::Response response = open();
  ASSERT_TRUE(response.accepted);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, duration_ns, 0.0));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  while (
    RollingControllerRtTestPeer::executionTime(*controller_) < low_water_boundary_ns)
  {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  EXPECT_EQ(
    RollingControllerRtTestPeer::executionTime(*controller_), low_water_boundary_ns);

  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kHolding);
  EXPECT_EQ(RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kLowWater);
  EXPECT_EQ(
    RollingControllerRtTestPeer::executionTime(*controller_), low_water_boundary_ns);
}

TEST_F(RtUpdateTest, ClockAnomalyUsesNominalStepsInsteadOfTheBadPeriod)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.accepted);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 500'000'000U, 0.1));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  for (std::size_t cycle = 0U; cycle < 10U; ++cycle) {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  const std::uint64_t execution_before =
    RollingControllerRtTestPeer::executionTime(*controller_);
  const auto command_before = command_positions_;

  ASSERT_EQ(update(9'000'000), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kStopping);
  EXPECT_EQ(
    RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kClockAnomaly);
  EXPECT_EQ(command_positions_, command_before);

  ASSERT_EQ(update(100'000'000), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::executionTime(*controller_),
    execution_before + kCycleNs);
  for (double command : command_positions_) {
    EXPECT_TRUE(std::isfinite(command));
  }
}

TEST_F(RtUpdateTest, BoundedPeriodJitterAdvancesWithoutClockAnomaly)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.accepted);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 500'000'000U, 0.0));
  ASSERT_EQ(update(), controller_interface::return_type::OK);

  constexpr std::array<std::int64_t, 6U> kPeriodsNs = {
    3'000'000, 5'000'000, 4'000'000, 7'000'000, 1'000'000, 4'000'000};
  std::uint64_t expected_execution_ns = 0U;
  for (const std::int64_t period_ns : kPeriodsNs) {
    ASSERT_EQ(update(period_ns), controller_interface::return_type::OK);
    expected_execution_ns += static_cast<std::uint64_t>(period_ns);
    EXPECT_EQ(
      RollingControllerRtTestPeer::executionTime(*controller_), expected_execution_ns);
    EXPECT_EQ(
      RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning);
    EXPECT_EQ(RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kNone);
  }
}

TEST_F(RtUpdateTest, NonPositivePeriodsLatchTheSameClockAnomaly)
{
  constexpr std::array<std::int64_t, 2> invalid_periods = {0, -1};
  std::uint8_t open_seed = 70U;
  std::uint8_t close_seed = 170U;
  for (const std::int64_t invalid_period : invalid_periods) {
    const Open::Response response = open(open_seed++);
    ASSERT_TRUE(response.accepted);
    RollingControllerRtTestPeer::submit(
      *controller_, makeTrajectory(response, 500'000'000U, 0.1));
    ASSERT_EQ(update(), controller_interface::return_type::OK);
    ASSERT_EQ(update(), controller_interface::return_type::OK);

    ASSERT_EQ(update(invalid_period), controller_interface::return_type::OK);
    EXPECT_EQ(
      RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kStopping);
    EXPECT_EQ(
      RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kClockAnomaly);
    for (std::size_t cycle = 0U;
      cycle < 100U &&
      RollingControllerRtTestPeer::sessionState(*controller_) == SessionState::kStopping;
      ++cycle)
    {
      ASSERT_EQ(update(), controller_interface::return_type::OK);
    }
    ASSERT_EQ(
      RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kHolding);
    const Close::Response finalized = RollingControllerRtTestPeer::close(
      *controller_,
      makeCloseRequest(response, close_seed++, Close::Request::FINALIZE));
    ASSERT_TRUE(finalized.accepted);
    ASSERT_TRUE(finalized.completed);
  }
}

TEST_F(RtUpdateTest, MovingGracefulCloseStopsContinuouslyAndRejectsLaterUpdates)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.accepted);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 500'000'000U, 0.1));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  for (std::size_t cycle = 0U; cycle < 10U; ++cycle) {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  const auto command_before_stop = command_positions_;
  const std::uint64_t execution_before_stop =
    RollingControllerRtTestPeer::executionTime(*controller_);

  const Close::Response stopping = RollingControllerRtTestPeer::close(
    *controller_, makeCloseRequest(response, 180U));
  ASSERT_TRUE(stopping.accepted);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(command_positions_, command_before_stop);
  EXPECT_EQ(
    RollingControllerRtTestPeer::executionTime(*controller_), execution_before_stop);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kStopping);
  EXPECT_EQ(
    RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kGracefulClose);

  Batch rejected = makeTrajectory(response, 500'000'000U, 0.2);
  rejected.sequence = 2U;
  RollingControllerRtTestPeer::submit(*controller_, rejected);
  EXPECT_EQ(
    RollingControllerRtTestPeer::lastReject(*controller_),
    RejectCode::kSessionNotAccepting);

  for (std::size_t cycle = 0U;
    cycle < 100U &&
    RollingControllerRtTestPeer::sessionState(*controller_) == SessionState::kStopping;
    ++cycle)
  {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      EXPECT_TRUE(std::isfinite(command_positions_[axis]));
      EXPECT_GE(command_positions_[axis], command_before_stop[axis]);
    }
  }
  ASSERT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kHolding);
  const Close::Response finalized = RollingControllerRtTestPeer::close(
    *controller_, makeCloseRequest(response, 181U, Close::Request::FINALIZE));
  EXPECT_TRUE(finalized.accepted);
  EXPECT_TRUE(finalized.completed);
}

TEST_F(RtUpdateTest, UpdateTimeoutStopsEvenWithAPlentifulOldFuture)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.accepted);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 500'000'000U, 0.1));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  RollingControllerRtTestPeer::expireAcceptedUpdate(*controller_);

  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kStopping);
  EXPECT_EQ(
    RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kUpdateTimeout);
}

TEST_F(RtUpdateTest, TimedOutSessionCanFinalizeAndOpenAFreshSession)
{
  const Open::Response first = open(90U);
  ASSERT_TRUE(first.accepted);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(first, 500'000'000U, 0.0));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  RollingControllerRtTestPeer::expireAcceptedUpdate(*controller_);
  for (std::size_t cycle = 0U;
    cycle < 100U &&
    RollingControllerRtTestPeer::sessionState(*controller_) != SessionState::kHolding;
    ++cycle)
  {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  ASSERT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kHolding);
  ASSERT_EQ(
    RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kUpdateTimeout);

  const Close::Response finalized = RollingControllerRtTestPeer::close(
    *controller_, makeCloseRequest(first, 190U, Close::Request::FINALIZE));
  ASSERT_TRUE(finalized.accepted);
  ASSERT_TRUE(finalized.completed);

  const Open::Response second = open(91U);
  ASSERT_TRUE(second.accepted);
  EXPECT_NE(second.session_id, first.session_id);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(second, 500'000'000U, 0.0));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning);
  EXPECT_EQ(RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kNone);
}

TEST_F(RtUpdateTest, PrimeTimeoutStopsAtTheOriginalHold)
{
  const auto initial_commands = command_positions_;
  ASSERT_TRUE(open().accepted);
  RollingControllerRtTestPeer::expirePrime(*controller_);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kHolding);
  EXPECT_EQ(
    RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kPrimeTimeout);
  EXPECT_EQ(command_positions_, initial_commands);
}

TEST_F(RtUpdateTest, UpdatePathDoesNotAllocate)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.accepted);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 500'000'000U, 0.05));
  const std::uint64_t before = tracked_allocation_count;
  track_allocations = true;
  for (std::size_t cycle = 0U; cycle < 100U; ++cycle) {
    if (update() != controller_interface::return_type::OK) {
      track_allocations = false;
      FAIL() << "update failed at cycle " << cycle;
    }
  }
  track_allocations = false;
  EXPECT_EQ(tracked_allocation_count, before);
}

TEST_F(RtUpdateTest, TenHertzKnotsAtThirtyHertzRemainStableAtFakeTwoHundredFiftyHertz)
{
  std::uint64_t cycle_count = 250U;
  if (const char * configured_cycles = std::getenv("E102_SOAK_CYCLES")) {
    char * end = nullptr;
    const unsigned long long parsed = std::strtoull(configured_cycles, &end, 10);
    ASSERT_NE(end, configured_cycles);
    ASSERT_EQ(*end, '\0');
    ASSERT_GT(parsed, 0U);
    cycle_count = static_cast<std::uint64_t>(parsed);
  }
  const bool realtime = []() {
      const char * value = std::getenv("E102_SOAK_REALTIME");
      return value != nullptr && std::string(value) == "1";
    }();

  const Open::Response response = open(100U);
  ASSERT_TRUE(response.accepted);
  std::uint64_t next_sequence = 1U;
  RollingControllerRtTestPeer::submit(
    *controller_, makeHoldWindow(response, next_sequence++, 0U));
  ASSERT_EQ(RollingControllerRtTestPeer::lastReject(*controller_), RejectCode::kNone);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning);

  const std::size_t expected_batch_count = static_cast<std::size_t>(
    (cycle_count * 30U) / 250U + 2U);
  std::vector<std::uint64_t> update_durations_ns;
  std::vector<std::uint64_t> validation_durations_ns;
  std::vector<std::uint64_t> callback_to_rt_ns;
  update_durations_ns.reserve(static_cast<std::size_t>(cycle_count));
  validation_durations_ns.reserve(expected_batch_count);
  callback_to_rt_ns.reserve(expected_batch_count);

  std::uint64_t publisher_accumulator = 0U;
  std::uint64_t published_batches = 1U;
  std::uint64_t late_replace_count = 0U;
  std::uint64_t allocation_count = 0U;
  std::uint64_t late_cycle_count = 0U;
  std::string failure_reason;
  auto last_submission_time = std::chrono::steady_clock::time_point{};
  bool submission_pending = false;
  const auto wall_start = std::chrono::steady_clock::now();
  auto next_deadline = wall_start;

  for (std::uint64_t cycle = 0U; cycle < cycle_count; ++cycle) {
    publisher_accumulator += 30U;
    if (publisher_accumulator >= 250U) {
      publisher_accumulator -= 250U;
      const std::uint64_t replace_from_ns =
        RollingControllerRtTestPeer::replaceableFrom(*controller_);
      Batch batch = makeHoldWindow(response, next_sequence++, replace_from_ns);
      const auto validation_start = std::chrono::steady_clock::now();
      RollingControllerRtTestPeer::submit(*controller_, batch);
      const auto validation_end = std::chrono::steady_clock::now();
      validation_durations_ns.push_back(
        static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            validation_end - validation_start).count()));
      last_submission_time = validation_end;
      submission_pending = true;
      ++published_batches;
      const RejectCode reject = RollingControllerRtTestPeer::lastReject(*controller_);
      if (reject == RejectCode::kLateReplace) {
        ++late_replace_count;
      }
      if (reject != RejectCode::kNone) {
        failure_reason = "batch_rejected_code=" +
          std::to_string(static_cast<unsigned int>(reject));
        break;
      }
    }

    const std::uint64_t allocations_before = tracked_allocation_count;
    track_allocations = true;
    const auto update_start = std::chrono::steady_clock::now();
    const controller_interface::return_type update_result = update();
    const auto update_end = std::chrono::steady_clock::now();
    track_allocations = false;
    allocation_count += tracked_allocation_count - allocations_before;
    update_durations_ns.push_back(
      static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          update_end - update_start).count()));
    if (submission_pending) {
      callback_to_rt_ns.push_back(
        static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            update_end - last_submission_time).count()));
      submission_pending = false;
    }
    if (update_result != controller_interface::return_type::OK) {
      failure_reason = "rt_update_returned_error";
      break;
    }
    if (allocation_count != 0U) {
      failure_reason = "rt_allocation_detected";
      break;
    }
    if (
      RollingControllerRtTestPeer::sessionState(*controller_) != SessionState::kRunning ||
      RollingControllerRtTestPeer::stopReason(*controller_) != StopReason::kNone)
    {
      failure_reason = "unexpected_stop_reason=" + std::to_string(
        static_cast<unsigned int>(
          RollingControllerRtTestPeer::stopReason(*controller_)));
      break;
    }
    for (const double command : command_positions_) {
      if (!std::isfinite(command)) {
        failure_reason = "non_finite_command";
        break;
      }
    }
    if (!failure_reason.empty()) {
      break;
    }

    if (realtime) {
      next_deadline += std::chrono::nanoseconds(kCycleNs);
      if (update_end > next_deadline) {
        ++late_cycle_count;
      } else {
        std::this_thread::sleep_until(next_deadline);
      }
    }
  }
  track_allocations = false;
  const auto wall_end = std::chrono::steady_clock::now();

  const std::uint64_t update_p50_ns =
    percentileNanoseconds(update_durations_ns, 0.50);
  const std::uint64_t update_p99_ns =
    percentileNanoseconds(update_durations_ns, 0.99);
  const std::uint64_t update_p999_ns =
    percentileNanoseconds(update_durations_ns, 0.999);
  const std::uint64_t validation_p50_ns =
    percentileNanoseconds(validation_durations_ns, 0.50);
  const std::uint64_t validation_p99_ns =
    percentileNanoseconds(validation_durations_ns, 0.99);
  const std::uint64_t validation_p999_ns =
    percentileNanoseconds(validation_durations_ns, 0.999);
  const std::uint64_t visibility_p999_ns =
    percentileNanoseconds(callback_to_rt_ns, 0.999);
  const std::uint64_t wall_duration_ms = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start).count());
  const bool passed = failure_reason.empty() &&
    update_durations_ns.size() == static_cast<std::size_t>(cycle_count);

  if (const char * report_path = std::getenv("E102_SOAK_REPORT")) {
    std::ofstream report(report_path, std::ios::out | std::ios::trunc);
    ASSERT_TRUE(report.is_open()) << "cannot open soak report: " << report_path;
    report << "{\n"
           << "  \"schema_version\": 1,\n"
           << "  \"scenario\": \"10hz_knots_30hz_batches_250hz_fake\",\n"
           << "  \"seed\": 57602,\n"
           << "  \"passed\": " << (passed ? "true" : "false") << ",\n"
           << "  \"realtime\": " << (realtime ? "true" : "false") << ",\n"
           << "  \"cycles_requested\": " << cycle_count << ",\n"
           << "  \"cycles_completed\": " << update_durations_ns.size() << ",\n"
           << "  \"simulated_duration_ns\": "
           << update_durations_ns.size() * kCycleNs << ",\n"
           << "  \"wall_duration_ms\": " << wall_duration_ms << ",\n"
           << "  \"published_batches\": " << published_batches << ",\n"
           << "  \"accepted_batches\": "
           << RollingControllerRtTestPeer::acceptedCount(*controller_) << ",\n"
           << "  \"rejected_batches\": "
           << RollingControllerRtTestPeer::rejectedCount(*controller_) << ",\n"
           << "  \"late_replace_count\": " << late_replace_count << ",\n"
           << "  \"rt_allocation_count\": " << allocation_count << ",\n"
           << "  \"invariant_failure_count\": "
           << RollingControllerRtTestPeer::invariantFailureCount(*controller_) << ",\n"
           << "  \"late_cycle_count\": " << late_cycle_count << ",\n"
           << "  \"max_point_count\": 6,\n"
           << "  \"knot_interval_ns\": 100000000,\n"
           << "  \"batch_rate_hz\": 30,\n"
           << "  \"update_rate_hz\": 250,\n"
           << "  \"update_p50_ns\": " << update_p50_ns << ",\n"
           << "  \"update_p99_ns\": " << update_p99_ns << ",\n"
           << "  \"update_p999_ns\": " << update_p999_ns << ",\n"
           << "  \"validation_p50_ns\": " << validation_p50_ns << ",\n"
           << "  \"validation_p99_ns\": " << validation_p99_ns << ",\n"
           << "  \"validation_p999_ns\": " << validation_p999_ns << ",\n"
           << "  \"callback_to_rt_p999_ns\": " << visibility_p999_ns << ",\n"
           << "  \"failure\": \"" << failure_reason << "\"\n"
           << "}\n";
  }

  EXPECT_TRUE(passed) << failure_reason;
  EXPECT_EQ(late_replace_count, 0U);
  EXPECT_EQ(allocation_count, 0U);
  EXPECT_EQ(RollingControllerRtTestPeer::rejectedCount(*controller_), 0U);
  EXPECT_EQ(RollingControllerRtTestPeer::invariantFailureCount(*controller_), 0U);
}

TEST_F(RtUpdateTest, ConcurrentOpenPublicationAndRtConsumptionRemainCoherent)
{
  std::atomic_bool rt_ready{false};
  std::atomic_bool start{false};
  std::atomic_bool prime_published{false};
  std::atomic_bool producer_done{false};
  std::atomic<std::uint64_t> update_errors{0U};
  std::thread rt_thread([&]() {
      rt_ready.store(true, std::memory_order_release);
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      if (update() != controller_interface::return_type::OK) {
        update_errors.fetch_add(1U, std::memory_order_relaxed);
      }
      while (!prime_published.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      std::size_t cycle = 1U;
      while (
        cycle < 20U &&
        (!producer_done.load(std::memory_order_acquire) || cycle < 10U))
      {
        if (update() != controller_interface::return_type::OK) {
          update_errors.fetch_add(1U, std::memory_order_relaxed);
        }
        ++cycle;
        std::this_thread::yield();
      }
    });

  while (!rt_ready.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  start.store(true, std::memory_order_release);
  const Open::Response response = open();
  if (!response.accepted) {
    prime_published.store(true, std::memory_order_release);
    producer_done.store(true, std::memory_order_release);
    rt_thread.join();
    FAIL() << "concurrent open was rejected with code " <<
      static_cast<unsigned int>(response.result.value);
  }
  Batch prime = makeTrajectory(response, 500'000'000U, 0.05);
  RollingControllerRtTestPeer::submit(*controller_, prime);
  prime_published.store(true, std::memory_order_release);
  for (std::uint64_t sequence = 2U; sequence <= 10U; ++sequence) {
    Batch candidate = makeTrajectory(response, 500'000'000U, 0.05);
    candidate.sequence = sequence;
    RollingControllerRtTestPeer::submit(*controller_, candidate);
  }
  producer_done.store(true, std::memory_order_release);
  rt_thread.join();

  ASSERT_EQ(update_errors.load(std::memory_order_relaxed), 0U);
  if (RollingControllerRtTestPeer::sessionState(*controller_) == SessionState::kPriming) {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning)
    << "stop_reason=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::stopReason(*controller_)) <<
    ", invariant_stage=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::invariantStage(*controller_));
  EXPECT_EQ(RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kNone);
  EXPECT_GE(RollingControllerRtTestPeer::activeGeneration(*controller_), 1U);
  for (const double command : command_positions_) {
    EXPECT_TRUE(std::isfinite(command));
  }
}

TEST_F(RtUpdateTest, ConcurrentRunningReplacementNeverTurnsAProtocolRejectIntoAStop)
{
  constexpr std::uint64_t duration_ns = 500'000'000U;
  constexpr double displacement = 0.05;
  const Open::Response response = open();
  ASSERT_TRUE(response.accepted);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, duration_ns, displacement));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning);

  std::atomic_bool start{false};
  std::atomic_bool producer_done{false};
  std::atomic<std::uint64_t> update_errors{0U};
  std::thread rt_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      std::size_t cycle = 0U;
      while (
        cycle < 100U &&
        (!producer_done.load(std::memory_order_acquire) || cycle < 40U))
      {
        if (update() != controller_interface::return_type::OK) {
          update_errors.fetch_add(1U, std::memory_order_relaxed);
        }
        ++cycle;
        std::this_thread::yield();
      }
    });

  start.store(true, std::memory_order_release);
  for (std::uint64_t sequence = 2U; sequence <= 80U; ++sequence) {
    const std::uint64_t replace_from_ns =
      RollingControllerRtTestPeer::replaceableFrom(*controller_);
    if (replace_from_ns >= duration_ns) {
      break;
    }
    RollingControllerRtTestPeer::submit(
      *controller_,
      makeReplacement(
        response, sequence, replace_from_ns, duration_ns,
        displacement, displacement));
    std::this_thread::yield();
  }
  producer_done.store(true, std::memory_order_release);
  rt_thread.join();

  EXPECT_EQ(update_errors.load(std::memory_order_relaxed), 0U);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning)
    << "stop_reason=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::stopReason(*controller_)) <<
    ", invariant_stage=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::invariantStage(*controller_));
  EXPECT_EQ(RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kNone);
  EXPECT_GE(RollingControllerRtTestPeer::activeGeneration(*controller_), 1U);
  for (const double command : command_positions_) {
    EXPECT_TRUE(std::isfinite(command));
  }
}

}  // namespace
}  // namespace rolling_trajectory_controller
