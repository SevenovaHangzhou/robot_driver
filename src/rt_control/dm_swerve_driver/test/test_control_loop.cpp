#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "dm_swerve_driver/control_loop.hpp"
#include "fake_can_transport.hpp"

namespace dm_swerve_driver {
namespace {

using namespace std::chrono_literals;

TEST(ControlLoopTest, StartupReadsRegistersSeedsPositionsAndEnablesAllMotors)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  std::vector<std::string> logs;
  ControlLoop loop{
    default_parameters(), std::move(transport),
    ControlLoopCallbacks{
      {},
      [&](DriverLogLevel, const std::string & message) {logs.push_back(message);}}};
  const auto start = std::chrono::steady_clock::time_point{};

  ASSERT_TRUE(loop.initialize(start));
  const auto status = loop.status();
  EXPECT_TRUE(status.initialized);
  EXPECT_TRUE(transport_observer->is_open());
  for (std::size_t index{0U}; index < status.motors.size(); ++index) {
    EXPECT_TRUE(status.motors[index].has_feedback);
    EXPECT_TRUE(status.motors[index].enabled());
    EXPECT_EQ(status.motors[index].seeded_from_multi_turn, index < kSwerveModuleCount);
  }
  EXPECT_TRUE(std::any_of(
      transport_observer->batches().begin(), transport_observer->batches().end(),
      [](const auto & batch) {return batch.size() == kMotorCount;}));
}

TEST(ControlLoopTest, MissingRegistersUseFallbackWithoutBlockingStartup)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  transport->set_register_replies_enabled(false);
  std::vector<std::string> warnings;
  ControlLoop loop{
    default_parameters(), std::move(transport),
    ControlLoopCallbacks{
      {},
      [&](DriverLogLevel level, const std::string & message) {
        if (level == DriverLogLevel::warning) {
          warnings.push_back(message);
        }
      }}};

  ASSERT_TRUE(loop.initialize({}));
  const auto status = loop.status();
  for (const auto & limits : status.motor_limits) {
    EXPECT_DOUBLE_EQ(limits.position_max, 12.5);
    EXPECT_DOUBLE_EQ(limits.velocity_max, 30.0);
    EXPECT_DOUBLE_EQ(limits.torque_max, 10.0);
  }
  EXPECT_FALSE(warnings.empty());
}

TEST(ControlLoopTest, CycleWritesOneEightFrameBatchAndRoutesFeedback)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  std::vector<ControlLoopOutput> outputs;
  ControlLoop loop{
    default_parameters(), std::move(transport),
    ControlLoopCallbacks{
      [&](const ControlLoopOutput & output) {outputs.push_back(output);}, {}}};
  const auto start = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(start));
  transport_observer->clear_batches();
  loop.submit_command(ChassisSpeeds{0.5, 0.0, 0.0}, start);
  loop.submit_imu_yaw(0.0, start);

  ASSERT_TRUE(loop.step(start + 10ms));
  ASSERT_EQ(transport_observer->batches().size(), 1U);
  EXPECT_EQ(transport_observer->batches().front().size(), kMotorCount);
  ASSERT_EQ(outputs.size(), 1U);
  EXPECT_DOUBLE_EQ(outputs.front().command.vx_mps, 0.5);
  EXPECT_EQ(loop.status().completed_cycles, 1U);
}

TEST(ControlLoopTest, UnknownAndMalformedFramesNeverEscapeCycle)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  ControlLoop loop{default_parameters(), std::move(transport)};
  ASSERT_TRUE(loop.initialize({}));

  const auto unknown = encode_motor_feedback(
    0x70U,
    MotorFeedback{1U, MotorError::enabled, 0.0, 0.0, 0.0, 30U, 30U},
    MotorLimits{12.5, 30.0, 10.0});
  transport_observer->inject(ReceivedCanFrame{unknown, {}});
  bool stepped{false};
  EXPECT_NO_THROW(stepped = loop.step(
      std::chrono::steady_clock::time_point{10ms}));
  EXPECT_TRUE(stepped);
  EXPECT_GE(loop.status().unknown_frames, 1U);
}

TEST(ControlLoopTest, OpenFailureIsReportedWithoutStarting)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  transport->set_fail_open(true);
  std::vector<std::string> errors;
  ControlLoop loop{
    default_parameters(), std::move(transport),
    ControlLoopCallbacks{
      {},
      [&](DriverLogLevel level, const std::string & message) {
        if (level == DriverLogLevel::error) {
          errors.push_back(message);
        }
      }}};

  EXPECT_FALSE(loop.initialize({}));
  EXPECT_FALSE(loop.status().initialized);
  EXPECT_FALSE(errors.empty());
}

TEST(ControlLoopTest, StopSendsDisableBatchAndClosesTransport)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  ControlLoop loop{default_parameters(), std::move(transport)};
  ASSERT_TRUE(loop.initialize({}));
  transport_observer->clear_batches();

  loop.stop();
  ASSERT_FALSE(transport_observer->is_open());
  ASSERT_FALSE(transport_observer->batches().empty());
  const auto & final_batch = transport_observer->batches().back();
  ASSERT_EQ(final_batch.size(), kMotorCount);
  for (const auto & frame : final_batch) {
    ASSERT_TRUE(decode_special_command(frame).has_value());
    EXPECT_EQ(*decode_special_command(frame), SpecialCommand::disable);
  }
}

TEST(ControlLoopTest, CommandAndImuDegradeThenRecoverWithoutLatch)
{
  auto parameters = default_parameters();
  parameters.control.cmd_vel_timeout_s = 0.05;
  parameters.odometry.imu_timeout_s = 0.02;
  auto transport = std::make_unique<test::FakeCanTransport>();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  loop.submit_command(ChassisSpeeds{0.5, 0.0, 0.0}, epoch);
  loop.submit_imu_yaw(0.0, epoch);

  ASSERT_TRUE(loop.step(epoch + 10ms));
  EXPECT_FALSE(loop.status().command_timed_out);
  EXPECT_FALSE(loop.status().imu_fallback);

  ASSERT_TRUE(loop.step(epoch + 100ms));
  const double fallback_yaw{loop.status().pose.heading_rad};
  EXPECT_TRUE(loop.status().command_timed_out);
  EXPECT_TRUE(loop.status().imu_fallback);

  loop.submit_command(ChassisSpeeds{0.25, 0.0, 0.0}, epoch + 110ms);
  loop.submit_imu_yaw(1.0, epoch + 110ms);
  ASSERT_TRUE(loop.step(epoch + 120ms));
  EXPECT_FALSE(loop.status().command_timed_out);
  EXPECT_FALSE(loop.status().imu_fallback);
  EXPECT_NEAR(loop.status().pose.heading_rad, fallback_yaw, 1e-9);
}

TEST(ControlLoopTest, SilentMotorIsReenabledAndAutomaticallyRejoins)
{
  auto parameters = default_parameters();
  parameters.safety.feedback_silent_cycles = 2U;
  parameters.safety.reenable_period_s = 0.01;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  transport_observer->clear_batches();
  transport_observer->set_dropped_mst_id(0x11U);

  ASSERT_TRUE(loop.step(epoch + 10ms));
  ASSERT_TRUE(loop.step(epoch + 20ms));
  const bool sent_enable = std::any_of(
    transport_observer->batches().begin(), transport_observer->batches().end(),
    [](const auto & batch) {
      return std::any_of(batch.begin(), batch.end(), [](const CanFrame & frame) {
        return frame.id == 1U &&
               decode_special_command(frame) == SpecialCommand::enable;
      });
    });
  EXPECT_TRUE(sent_enable);
  EXPECT_GE(loop.status().motors[0].consecutive_missed_frames, 2U);

  transport_observer->set_dropped_mst_id(std::nullopt);
  ASSERT_TRUE(loop.step(epoch + 30ms));
  EXPECT_EQ(loop.status().motors[0].consecutive_missed_frames, 0U);
}

TEST(ControlLoopTest, MotorFaultSchedulesClearAndReenable)
{
  auto parameters = default_parameters();
  parameters.safety.reenable_period_s = 0.01;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  transport_observer->clear_batches();
  transport_observer->override_next_error(0x11U, MotorError::over_current);

  ASSERT_TRUE(loop.step(epoch + 10ms));
  bool sent_clear{false};
  bool sent_enable{false};
  for (const auto & batch : transport_observer->batches()) {
    for (const auto & frame : batch) {
      if (frame.id != 1U) {
        continue;
      }
      sent_clear = sent_clear || decode_special_command(frame) == SpecialCommand::clear_fault;
      sent_enable = sent_enable || decode_special_command(frame) == SpecialCommand::enable;
    }
  }
  EXPECT_TRUE(sent_clear);
  EXPECT_TRUE(sent_enable);
}

TEST(ControlLoopTest, WholeBusSilenceGatesDriveAndFeedbackRestoresIt)
{
  auto parameters = default_parameters();
  parameters.safety.feedback_silent_cycles = 2U;
  parameters.safety.reenable_period_s = 0.01;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  loop.submit_command(ChassisSpeeds{0.5, 0.0, 0.0}, epoch);
  transport_observer->set_drop_all_feedback(true);
  ASSERT_TRUE(loop.step(epoch + 10ms));
  ASSERT_TRUE(loop.step(epoch + 20ms));
  EXPECT_TRUE(loop.status().bus_silent);

  transport_observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 30ms));
  ASSERT_FALSE(transport_observer->batches().empty());
  const auto & gated_commands = transport_observer->batches().front();
  for (const auto & frame : gated_commands) {
    if (frame.id >= 5U && frame.id <= 8U) {
      const auto command = decode_mit_command(frame, parameters.limits_fallback);
      EXPECT_NEAR(command.velocity, 0.0, 0.02);
    }
  }

  transport_observer->set_drop_all_feedback(false);
  ASSERT_TRUE(loop.step(epoch + 40ms));
  EXPECT_FALSE(loop.status().bus_silent);
  transport_observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 50ms));
  const auto & restored_commands = transport_observer->batches().front();
  const auto drive = std::find_if(
    restored_commands.begin(), restored_commands.end(),
    [](const CanFrame & frame) {return frame.id == 5U;});
  ASSERT_NE(drive, restored_commands.end());
  EXPECT_GT(decode_mit_command(*drive, parameters.limits_fallback).velocity, 0.1);
}

TEST(ControlLoopTest, ManualClearFaultsIsExecutedByControlThread)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  ControlLoop loop{default_parameters(), std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  transport_observer->clear_batches();

  loop.request_clear_faults();
  ASSERT_TRUE(loop.step(epoch + 10ms));
  const bool sent_clear = std::any_of(
    transport_observer->batches().begin(), transport_observer->batches().end(),
    [](const auto & batch) {
      return std::any_of(batch.begin(), batch.end(), [](const CanFrame & frame) {
        return decode_special_command(frame) == SpecialCommand::clear_fault;
      });
    });
  EXPECT_TRUE(sent_clear);
}

TEST(ControlLoopTest, TimeoutCanReturnSteeringTargetsToZeroInsteadOfHolding)
{
  auto parameters = default_parameters();
  parameters.control.cmd_vel_timeout_s = 0.02;
  parameters.control.hold_steer_on_timeout = false;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));

  for (int cycle{1}; cycle <= 100; ++cycle) {
    const auto now = epoch + cycle * 10ms;
    loop.submit_command(ChassisSpeeds{0.0, 0.5, 0.0}, now);
    ASSERT_TRUE(loop.step(now));
  }
  transport_observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 1030ms));

  ASSERT_FALSE(transport_observer->batches().empty());
  const auto & frames = transport_observer->batches().front();
  const auto steering = std::find_if(
    frames.begin(), frames.end(), [](const CanFrame & frame) {return frame.id == 1U;});
  ASSERT_NE(steering, frames.end());
  const auto command = decode_mit_command(*steering, parameters.limits_fallback);
  EXPECT_NEAR(command.position, 0.0, 1e-3);
}

}  // namespace
}  // namespace dm_swerve_driver
