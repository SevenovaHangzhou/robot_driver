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
  std::size_t timeout_writes{0U};
  for (const auto & batch : transport_observer->batches()) {
    for (const auto & frame : batch) {
      if (frame.id != kRegisterCanId) {
        continue;
      }
      const auto request = decode_register_reply(frame);
      if (request.operation == RegisterOperation::write &&
        request.register_id == static_cast<std::uint8_t>(RegisterId::timeout))
      {
        ++timeout_writes;
      }
    }
  }
  EXPECT_EQ(timeout_writes, kMotorCount);
}

TEST(ControlLoopTest, MissingLimitRegistersRejectStrictStartup)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  transport->set_register_replies_enabled(false);
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

TEST(ControlLoopTest, ExplicitVcanFallbackAllowsMissingLimitRegisters)
{
  auto parameters = default_parameters();
  parameters.can.allow_fallback_limits = true;
  auto transport = std::make_unique<test::FakeCanTransport>();
  transport->set_register_replies_enabled(false);
  ControlLoop loop{parameters, std::move(transport)};

  EXPECT_TRUE(loop.initialize({}));
}

TEST(ControlLoopTest, MissingMotorFeedbackRejectsStartup)
{
  auto parameters = default_parameters();
  parameters.can.allow_fallback_limits = true;
  auto transport = std::make_unique<test::FakeCanTransport>();
  transport->set_dropped_mst_id(0x11U);
  ControlLoop loop{parameters, std::move(transport)};

  EXPECT_FALSE(loop.initialize({}));
}

TEST(ControlLoopTest, UnconfirmedMotorEnableRejectsStartup)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  transport->set_ignored_enable_esc_id(2U);
  ControlLoop loop{default_parameters(), std::move(transport)};

  EXPECT_FALSE(loop.initialize({}));
}

TEST(ControlLoopTest, GearedSteeringRequiresConsistentMultiTurnPosition)
{
  auto parameters = default_parameters();
  parameters.steering.gear_ratio = 2.0;

  auto missing = std::make_unique<test::FakeCanTransport>();
  missing->set_suppressed_register(RegisterId::multi_turn_position);
  std::vector<std::string> missing_logs;
  ControlLoop missing_loop{
    parameters, std::move(missing),
    ControlLoopCallbacks{{}, [&](DriverLogLevel, const std::string & message) {
      missing_logs.push_back(message);
    }}};
  EXPECT_FALSE(missing_loop.initialize({}));
  EXPECT_TRUE(std::any_of(missing_logs.begin(), missing_logs.end(), [](const auto & message) {
    return message.find("rezero") != std::string::npos;
  }));

  auto inconsistent = std::make_unique<test::FakeCanTransport>();
  inconsistent->set_multi_turn_override(1U, 1.0F);
  ControlLoop inconsistent_loop{parameters, std::move(inconsistent)};
  EXPECT_FALSE(inconsistent_loop.initialize({}));
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
  loop.submit_imu_yaw(0.1, epoch + 110ms);
  ASSERT_TRUE(loop.step(epoch + 120ms));
  EXPECT_FALSE(loop.status().command_timed_out);
  EXPECT_FALSE(loop.status().imu_fallback);
  EXPECT_NEAR(loop.status().pose.heading_rad, fallback_yaw, 1e-9);
}

TEST(ControlLoopTest, SingleSilentMotorGatesEveryDriveUntilItRejoins)
{
  auto parameters = default_parameters();
  parameters.safety.feedback_silent_cycles = 2U;
  parameters.safety.reenable_period_s = 1.0;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  loop.submit_command(ChassisSpeeds{0.5, 0.0, 0.0}, epoch);
  transport_observer->clear_batches();
  transport_observer->set_dropped_mst_id(0x11U);

  ASSERT_TRUE(loop.step(epoch + 10ms));
  ASSERT_TRUE(loop.step(epoch + 20ms));
  EXPECT_TRUE(loop.status().faulted);
  EXPECT_FALSE(loop.status().fault_latched);
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

  transport_observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 30ms));
  ASSERT_FALSE(transport_observer->batches().empty());
  for (const auto & frame : transport_observer->batches().front()) {
    if (frame.id >= 5U && frame.id <= 8U) {
      EXPECT_NEAR(
        decode_mit_command(frame, parameters.limits_fallback).velocity, 0.0, 0.02);
    }
  }

  transport_observer->set_dropped_mst_id(std::nullopt);
  ASSERT_TRUE(loop.step(epoch + 40ms));
  EXPECT_EQ(loop.status().motors[0].consecutive_missed_frames, 0U);
  EXPECT_FALSE(loop.status().faulted);

  transport_observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 50ms));
  const auto drive = std::find_if(
    transport_observer->batches().front().begin(),
    transport_observer->batches().front().end(),
    [](const CanFrame & frame) {return frame.id == 5U;});
  ASSERT_NE(drive, transport_observer->batches().front().end());
  EXPECT_GT(decode_mit_command(*drive, parameters.limits_fallback).velocity, 0.1);
}

TEST(ControlLoopTest, OverCurrentLatchesAndNeverAutoClears)
{
  auto parameters = default_parameters();
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  transport_observer->clear_batches();
  transport_observer->override_next_error(0x11U, MotorError::over_current);

  ASSERT_TRUE(loop.step(epoch + 10ms));
  EXPECT_TRUE(loop.status().faulted);
  EXPECT_TRUE(loop.status().fault_latched);
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
  EXPECT_FALSE(sent_clear);
  EXPECT_FALSE(sent_enable);

  transport_observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 20ms));
  for (const auto & frame : transport_observer->batches().front()) {
    if (frame.id >= 5U && frame.id <= 8U) {
      EXPECT_NEAR(
        decode_mit_command(frame, parameters.limits_fallback).velocity, 0.0, 0.02);
    }
  }
}

TEST(ControlLoopTest, FaultedCycleHoldsSteeringInsteadOfFollowingNewCommand)
{
  auto parameters = default_parameters();
  parameters.steering.max_slew_radps = 1000.0;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));

  loop.submit_command(ChassisSpeeds{0.0, 0.5, 0.0}, epoch);
  observer->override_next_error(0x11U, MotorError::over_current);
  observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 10ms));
  ASSERT_TRUE(loop.status().fault_latched);
  observer->clear_batches();
  loop.submit_command(ChassisSpeeds{0.5, 0.0, 0.0}, epoch + 10ms);
  ASSERT_TRUE(loop.step(epoch + 20ms));
  const auto first_held_steering = std::find_if(
    observer->batches().back().begin(), observer->batches().back().end(),
    [](const CanFrame & frame) {return frame.id == 1U;});
  ASSERT_NE(first_held_steering, observer->batches().back().end());
  const auto first_held_position = decode_mit_command(
    *first_held_steering, parameters.limits_fallback).position;

  observer->clear_batches();
  loop.submit_command(ChassisSpeeds{0.0, -0.5, 0.0}, epoch + 20ms);
  ASSERT_TRUE(loop.step(epoch + 30ms));
  const auto second_held_steering = std::find_if(
    observer->batches().back().begin(), observer->batches().back().end(),
    [](const CanFrame & frame) {return frame.id == 1U;});
  ASSERT_NE(second_held_steering, observer->batches().back().end());
  const auto second_held_position = decode_mit_command(
    *second_held_steering, parameters.limits_fallback).position;

  EXPECT_NEAR(second_held_position, first_held_position, 0.05);
}

TEST(ControlLoopTest, UnderVoltageSchedulesRateLimitedClearAndReenable)
{
  auto parameters = default_parameters();
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * transport_observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  transport_observer->clear_batches();
  transport_observer->override_next_error(0x11U, MotorError::under_voltage);

  ASSERT_TRUE(loop.step(epoch + 10ms));
  bool sent_clear{false};
  bool sent_enable{false};
  for (const auto & batch : transport_observer->batches()) {
    for (const auto & frame : batch) {
      if (frame.id == 1U) {
        sent_clear = sent_clear ||
          decode_special_command(frame) == SpecialCommand::clear_fault;
        sent_enable = sent_enable || decode_special_command(frame) == SpecialCommand::enable;
      }
    }
  }
  EXPECT_TRUE(sent_clear);
  EXPECT_TRUE(sent_enable);
  EXPECT_TRUE(loop.status().faulted);
  EXPECT_FALSE(loop.status().fault_latched);
}

TEST(ControlLoopTest, WholeBusSilenceGatesDriveAndFeedbackRestoresIt)
{
  auto parameters = default_parameters();
  parameters.safety.feedback_silent_cycles = 2U;
  parameters.safety.reenable_period_s = 1.0;
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
  transport_observer->override_next_error(0x11U, MotorError::over_current);
  ASSERT_TRUE(loop.step(epoch + 10ms));
  ASSERT_TRUE(loop.status().fault_latched);
  transport_observer->clear_batches();
  transport_observer->set_dropped_mst_id(0x12U);

  loop.request_clear_faults();
  ASSERT_TRUE(loop.step(epoch + 20ms));
  const bool sent_clear = std::any_of(
    transport_observer->batches().begin(), transport_observer->batches().end(),
    [](const auto & batch) {
      return std::any_of(batch.begin(), batch.end(), [](const CanFrame & frame) {
        return decode_special_command(frame) == SpecialCommand::clear_fault;
      });
    });
  EXPECT_TRUE(sent_clear);
  EXPECT_TRUE(loop.status().fault_latched);
  EXPECT_TRUE(loop.status().faulted);

  transport_observer->set_dropped_mst_id(std::nullopt);
  loop.request_clear_faults();
  ASSERT_TRUE(loop.step(epoch + 30ms));
  EXPECT_FALSE(loop.status().fault_latched);
  EXPECT_FALSE(loop.status().faulted);
}

TEST(ControlLoopTest, TimeoutCanReturnSteeringTargetsToZeroInsteadOfHolding)
{
  auto parameters = default_parameters();
  parameters.control.cmd_vel_timeout_s = 0.02;
  parameters.control.hold_steer_on_timeout = false;
  parameters.steering.max_slew_radps = 1000.0;
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

TEST(ControlLoopTest, TimeoutHoldKeepsCurrentSteeringTargetAndSuppressesFeedforward)
{
  auto parameters = default_parameters();
  parameters.control.cmd_vel_timeout_s = 0.02;
  parameters.control.hold_steer_on_timeout = true;
  parameters.steering.max_slew_radps = 1000.0;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  std::vector<ControlLoopOutput> outputs;
  ControlLoop loop{
    parameters, std::move(transport),
    ControlLoopCallbacks{
      [&](const ControlLoopOutput & output) {outputs.push_back(output);}, {}}};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));

  loop.submit_command(ChassisSpeeds{0.0, 0.5, 0.0}, epoch);
  observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 10ms));
  ASSERT_FALSE(outputs.empty());
  const auto measured_angle = outputs.back().steering_angle_rad[0];
  const auto moving = std::find_if(
    observer->batches().back().begin(), observer->batches().back().end(),
    [](const CanFrame & frame) {return frame.id == 1U;});
  ASSERT_NE(moving, observer->batches().back().end());

  observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 100ms));
  const auto held = std::find_if(
    observer->batches().back().begin(), observer->batches().back().end(),
    [](const CanFrame & frame) {return frame.id == 1U;});
  ASSERT_NE(held, observer->batches().back().end());
  const auto held_command = decode_mit_command(*held, parameters.limits_fallback);

  EXPECT_TRUE(loop.status().command_timed_out);
  EXPECT_NEAR(held_command.position, measured_angle, 0.05);
  EXPECT_NEAR(held_command.velocity, 0.0, 0.02);
}

TEST(ControlLoopTest, PublishesMeasuredTwistInsteadOfNonzeroCommandDuringMotion)
{
  auto parameters = default_parameters();
  parameters.control.cmd_vel_timeout_s = 1.0;
  std::vector<ControlLoopOutput> outputs;
  ControlLoop loop{
    parameters,
    std::make_unique<test::FakeCanTransport>(),
    ControlLoopCallbacks{
      [&](const ControlLoopOutput & output) {outputs.push_back(output);}, {}}};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));

  for (int cycle{1}; cycle <= 20; ++cycle) {
    const auto now = epoch + cycle * 10ms;
    loop.submit_command(ChassisSpeeds{0.5, 0.0, 0.0}, now);
    loop.submit_imu_yaw(0.0, now, 0.7);
    ASSERT_TRUE(loop.step(now));
  }

  ASSERT_FALSE(outputs.empty());
  const auto & output = outputs.back();
  EXPECT_GT(output.measured_twist.vx_mps, 0.05);
  EXPECT_NEAR(output.measured_twist.vy_mps, 0.0, 0.02);
  EXPECT_NEAR(output.measured_twist.omega_radps, 0.7, 1e-12);
}

TEST(ControlLoopTest, AlignmentGateReportsZeroMeasuredTwistForNonzeroCommand)
{
  std::vector<ControlLoopOutput> outputs;
  ControlLoop loop{
    default_parameters(),
    std::make_unique<test::FakeCanTransport>(),
    ControlLoopCallbacks{
      [&](const ControlLoopOutput & output) {outputs.push_back(output);}, {}}};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  loop.submit_command(ChassisSpeeds{0.0, 0.5, 0.0}, epoch);
  loop.submit_imu_yaw(0.0, epoch, 0.0);

  ASSERT_TRUE(loop.step(epoch + 10ms));
  ASSERT_EQ(outputs.size(), 1U);
  EXPECT_TRUE(outputs.front().alignment_gated);
  EXPECT_GT(outputs.front().command.vy_mps, 0.1);
  EXPECT_NEAR(outputs.front().measured_twist.vx_mps, 0.0, 0.02);
  EXPECT_NEAR(outputs.front().measured_twist.vy_mps, 0.0, 0.02);
}

TEST(ControlLoopTest, DesaturatesAllModulesToLowestMotorVelocityCapability)
{
  std::array<MotorLimits, kMotorCount> limits{};
  limits.fill(MotorLimits{12.5, 30.0, 10.0});
  limits[kSwerveModuleCount] = MotorLimits{12.5, 5.0, 10.0};
  auto parameters = default_parameters();
  parameters.chassis.max_wheel_acceleration_mps2 = 1000.0;
  auto transport = std::make_unique<test::FakeCanTransport>(limits);
  auto * observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  observer->clear_batches();
  loop.submit_command(ChassisSpeeds{1.0, 0.0, 0.0}, epoch);

  ASSERT_TRUE(loop.step(epoch + 10ms));
  ASSERT_FALSE(observer->batches().empty());
  const auto & command_batch = observer->batches().front();
  for (std::size_t module{0U}; module < kSwerveModuleCount; ++module) {
    const std::uint16_t drive_id{static_cast<std::uint16_t>(5U + module)};
    const auto frame = std::find_if(
      command_batch.begin(), command_batch.end(),
      [&](const CanFrame & candidate) {return candidate.id == drive_id;});
    ASSERT_NE(frame, command_batch.end());
    const auto command = decode_mit_command(
      *frame, limits[module + kSwerveModuleCount]);
    EXPECT_NEAR(command.velocity, 5.0, 0.05);
  }
}

TEST(ControlLoopTest, UsesClampedMeasuredCyclePeriodForDriveAcceleration)
{
  auto parameters = default_parameters();
  parameters.chassis.max_wheel_acceleration_mps2 = 10.0;
  parameters.control.cmd_vel_timeout_s = 1.0;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  loop.submit_command(ChassisSpeeds{1.0, 0.0, 0.0}, epoch);
  observer->clear_batches();

  ASSERT_TRUE(loop.step(epoch + 5ms));
  const auto first = std::find_if(
    observer->batches().front().begin(), observer->batches().front().end(),
    [](const CanFrame & frame) {return frame.id == 5U;});
  ASSERT_NE(first, observer->batches().front().end());
  EXPECT_NEAR(
    decode_mit_command(*first, parameters.limits_fallback).velocity,
    0.5, 0.03);

  observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 100ms));
  const auto second = std::find_if(
    observer->batches().front().begin(), observer->batches().front().end(),
    [](const CanFrame & frame) {return frame.id == 5U;});
  ASSERT_NE(second, observer->batches().front().end());
  EXPECT_NEAR(
    decode_mit_command(*second, parameters.limits_fallback).velocity,
    2.5, 0.03);
}

TEST(ControlLoopTest, SteeringCommandSlewIsAppliedBeforeAlignment)
{
  auto parameters = default_parameters();
  parameters.steering.max_slew_radps = 1.0;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  observer->clear_batches();
  loop.submit_command(ChassisSpeeds{0.0, 0.5, 0.0}, epoch);

  ASSERT_TRUE(loop.step(epoch + 10ms));
  ASSERT_FALSE(observer->batches().empty());
  const auto steering = std::find_if(
    observer->batches().front().begin(), observer->batches().front().end(),
    [](const CanFrame & frame) {return frame.id == 1U;});
  ASSERT_NE(steering, observer->batches().front().end());
  const auto command = decode_mit_command(*steering, parameters.limits_fallback);
  EXPECT_NEAR(command.position, 0.01, 1e-3);
}

TEST(ControlLoopTest, RejectsImplausibleImuYawJumpButAcceptsWrapCrossing)
{
  auto parameters = default_parameters();
  parameters.odometry.max_imu_yaw_step_rad = 0.5;
  ControlLoop loop{
    parameters, std::make_unique<test::FakeCanTransport>()};
  const auto epoch = std::chrono::steady_clock::time_point{};

  EXPECT_TRUE(loop.submit_imu_yaw(0.0, epoch));
  EXPECT_FALSE(loop.submit_imu_yaw(1.0, epoch + 10ms));

  ControlLoop wrapping{
    parameters, std::make_unique<test::FakeCanTransport>()};
  EXPECT_TRUE(wrapping.submit_imu_yaw(kPi - 0.01, epoch));
  EXPECT_TRUE(wrapping.submit_imu_yaw(-kPi + 0.01, epoch + 10ms));
}

TEST(ControlLoopTest, AcceptsImuRecoveryAfterAProlongedGap)
{
  auto parameters = default_parameters();
  parameters.odometry.imu_timeout_s = 0.05;
  parameters.odometry.max_imu_yaw_step_rad = 0.5;
  ControlLoop loop{
    parameters, std::make_unique<test::FakeCanTransport>()};
  const auto epoch = std::chrono::steady_clock::time_point{};

  EXPECT_TRUE(loop.submit_imu_yaw(0.0, epoch));
  EXPECT_TRUE(loop.submit_imu_yaw(1.0, epoch + 1s));
  EXPECT_TRUE(loop.submit_imu_yaw(1.1, epoch + 1010ms));
}

TEST(ControlLoopTest, RejectsOutOfOrderImuTimestamp)
{
  ControlLoop loop{
    default_parameters(), std::make_unique<test::FakeCanTransport>()};
  const auto epoch = std::chrono::steady_clock::time_point{};

  EXPECT_TRUE(loop.submit_imu_yaw(0.0, epoch + 20ms));
  EXPECT_FALSE(loop.submit_imu_yaw(0.1, epoch + 10ms));
  EXPECT_TRUE(loop.submit_imu_yaw(0.1, epoch + 30ms));
}

TEST(ControlLoopTest, RejectsOutOfOrderCommandTimestamp)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  ControlLoop loop{default_parameters(), std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};

  ASSERT_TRUE(loop.initialize(epoch));
  loop.submit_command(ChassisSpeeds{0.5, 0.0, 0.0}, epoch + 20ms);
  loop.submit_command(ChassisSpeeds{0.0, 0.5, 0.0}, epoch + 10ms);
  observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 25ms));
  const auto drive = std::find_if(
    observer->batches().back().begin(), observer->batches().back().end(),
    [](const CanFrame & frame) {return frame.id == 5U;});
  ASSERT_NE(drive, observer->batches().back().end());
  EXPECT_GT(decode_mit_command(*drive, default_parameters().limits_fallback).velocity, 0.1);
}

TEST(ControlLoopTest, PublishesCurrentOdometryQualityInputs)
{
  auto parameters = default_parameters();
  std::vector<ControlLoopOutput> outputs;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  ControlLoop loop{
    parameters, std::move(transport),
    ControlLoopCallbacks{
      [&](const ControlLoopOutput & output) {outputs.push_back(output);}, {}}};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  observer->set_dropped_mst_id(0x11U);
  ASSERT_TRUE(loop.submit_imu_yaw(0.0, epoch));

  ASSERT_TRUE(loop.step(epoch + 10ms));

  ASSERT_FALSE(outputs.empty());
  EXPECT_FALSE(outputs.back().imu_fallback);
  EXPECT_EQ(outputs.back().valid_module_count, 3U);
}

TEST(ControlLoopTest, HealthyImuAngularVelocityDoesNotDependOnWheelFk)
{
  auto parameters = default_parameters();
  std::vector<ControlLoopOutput> outputs;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  ControlLoop loop{
    parameters, std::move(transport),
    ControlLoopCallbacks{
      [&](const ControlLoopOutput & output) {outputs.push_back(output);}, {}}};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  observer->set_drop_all_feedback(true);
  ASSERT_TRUE(loop.submit_imu_yaw(0.0, epoch, 0.7));

  ASSERT_TRUE(loop.step(epoch + 10ms));

  ASSERT_FALSE(outputs.empty());
  EXPECT_FALSE(outputs.back().imu_fallback);
  EXPECT_EQ(outputs.back().valid_module_count, 0U);
  EXPECT_DOUBLE_EQ(outputs.back().measured_twist.vx_mps, 0.0);
  EXPECT_DOUBLE_EQ(outputs.back().measured_twist.vy_mps, 0.0);
  EXPECT_DOUBLE_EQ(outputs.back().measured_twist.omega_radps, 0.7);
}

TEST(ControlLoopTest, TransportExceptionGatesUntilACompleteFeedbackCycle)
{
  auto parameters = default_parameters();
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  ControlLoop loop{parameters, std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  loop.submit_command(ChassisSpeeds{0.5, 0.0, 0.0}, epoch);
  observer->clear_batches();
  observer->set_fail_io(true);

  EXPECT_FALSE(loop.step(epoch + 10ms));
  EXPECT_TRUE(loop.status().transport_faulted);
  EXPECT_TRUE(loop.status().faulted);

  observer->set_fail_io(false);
  ASSERT_TRUE(loop.step(epoch + 20ms));
  EXPECT_FALSE(loop.status().transport_faulted);
  ASSERT_FALSE(observer->batches().empty());
  const auto drive = std::find_if(
    observer->batches().back().begin(), observer->batches().back().end(),
    [](const CanFrame & frame) {return frame.id == 5U;});
  ASSERT_NE(drive, observer->batches().back().end());
  EXPECT_NEAR(decode_mit_command(*drive, parameters.limits_fallback).velocity, 0.0, 0.02);

  observer->clear_batches();
  ASSERT_TRUE(loop.step(epoch + 30ms));
  const auto resumed_drive = std::find_if(
    observer->batches().back().begin(), observer->batches().back().end(),
    [](const CanFrame & frame) {return frame.id == 5U;});
  ASSERT_NE(resumed_drive, observer->batches().back().end());
  EXPECT_GT(decode_mit_command(*resumed_drive, parameters.limits_fallback).velocity, 0.1);
}

TEST(ControlLoopTest, HardwareErrorFrameIsExcludedFromMeasuredModuleQuality)
{
  auto parameters = default_parameters();
  std::vector<ControlLoopOutput> outputs;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  ControlLoop loop{
    parameters, std::move(transport),
    ControlLoopCallbacks{
      [&](const ControlLoopOutput & output) {outputs.push_back(output);}, {}}};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  observer->override_next_error(0x11U, MotorError::over_current);
  ASSERT_TRUE(loop.submit_imu_yaw(0.0, epoch));

  ASSERT_TRUE(loop.step(epoch + 10ms));

  ASSERT_FALSE(outputs.empty());
  EXPECT_EQ(outputs.back().valid_module_count, 3U);
  EXPECT_TRUE(loop.status().fault_latched);
}

TEST(ControlLoopTest, RecoverableErrorFrameIsExcludedBeforeRecoveryAction)
{
  auto parameters = default_parameters();
  std::vector<ControlLoopOutput> outputs;
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  ControlLoop loop{
    parameters, std::move(transport),
    ControlLoopCallbacks{
      [&](const ControlLoopOutput & output) {outputs.push_back(output);}, {}}};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  observer->override_next_error(0x11U, MotorError::under_voltage);
  ASSERT_TRUE(loop.submit_imu_yaw(0.0, epoch));

  ASSERT_TRUE(loop.step(epoch + 10ms));

  ASSERT_FALSE(outputs.empty());
  EXPECT_EQ(outputs.back().valid_module_count, 3U);
  EXPECT_FALSE(loop.status().fault_latched);
}

TEST(ControlLoopTest, CollectExceptionAlsoEntersTransportSafetyWindow)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  ControlLoop loop{default_parameters(), std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  observer->set_fail_collect(true);

  EXPECT_FALSE(loop.step(epoch + 10ms));
  EXPECT_TRUE(loop.status().transport_faulted);
  EXPECT_TRUE(loop.status().faulted);

  observer->set_fail_collect(false);
  ASSERT_TRUE(loop.step(epoch + 20ms));
  EXPECT_FALSE(loop.status().transport_faulted);
}

TEST(ControlLoopTest, ClosedTransportAfterInitializationEntersSafetyFault)
{
  auto transport = std::make_unique<test::FakeCanTransport>();
  auto * observer = transport.get();
  ControlLoop loop{default_parameters(), std::move(transport)};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));
  observer->force_close();

  EXPECT_FALSE(loop.step(epoch + 10ms));
  EXPECT_TRUE(loop.status().transport_faulted);
  EXPECT_TRUE(loop.status().faulted);
}

}  // namespace
}  // namespace dm_swerve_driver
