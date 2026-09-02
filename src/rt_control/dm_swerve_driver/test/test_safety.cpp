#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>

#include "dm_swerve_driver/safety_monitor.hpp"

namespace dm_swerve_driver {
namespace {

using namespace std::chrono_literals;

const std::array<Translation2d, kSwerveModuleCount> kLocations{
  Translation2d{0.5, 0.4}, Translation2d{0.5, -0.4},
  Translation2d{-0.5, 0.4}, Translation2d{-0.5, -0.4}};

TEST(SafetyMonitorTest, CommandTimeoutStopsAndNewCommandImmediatelyRecovers)
{
  SafetyMonitor monitor{default_parameters()};
  const auto epoch = std::chrono::steady_clock::time_point{};
  const ChassisSpeeds command{1.0, -0.2, 0.3};

  auto decision = monitor.command_for_cycle(command, epoch, epoch + 100ms);
  EXPECT_FALSE(decision.timed_out);
  EXPECT_TRUE(decision.state_changed);
  EXPECT_DOUBLE_EQ(decision.command.vx_mps, 1.0);

  decision = monitor.command_for_cycle(command, epoch, epoch + 300ms);
  EXPECT_TRUE(decision.timed_out);
  EXPECT_TRUE(decision.state_changed);
  EXPECT_DOUBLE_EQ(decision.command.vx_mps, 0.0);

  decision = monitor.command_for_cycle(command, epoch + 310ms, epoch + 320ms);
  EXPECT_FALSE(decision.timed_out);
  EXPECT_TRUE(decision.state_changed);
  EXPECT_DOUBLE_EQ(decision.command.vx_mps, 1.0);
}

TEST(SafetyMonitorTest, ImuFallbackAndRecoveryKeepYawContinuous)
{
  SafetyMonitor monitor{default_parameters()};
  const auto epoch = std::chrono::steady_clock::time_point{};

  auto yaw = monitor.update_yaw(TimedYawSample{1.0, epoch}, 0.0, epoch);
  EXPECT_FALSE(yaw.imu_fallback);
  EXPECT_NEAR(yaw.yaw_rad, 1.0, 1e-12);

  yaw = monitor.update_yaw(TimedYawSample{1.0, epoch}, 0.1, epoch + 300ms);
  EXPECT_TRUE(yaw.imu_fallback);
  EXPECT_TRUE(yaw.source_changed);
  EXPECT_NEAR(yaw.yaw_rad, 1.1, 1e-12);

  yaw = monitor.update_yaw(std::nullopt, 0.2, epoch + 310ms);
  EXPECT_NEAR(yaw.yaw_rad, 1.3, 1e-12);

  yaw = monitor.update_yaw(TimedYawSample{2.0, epoch + 320ms}, 0.0, epoch + 320ms);
  EXPECT_FALSE(yaw.imu_fallback);
  EXPECT_TRUE(yaw.source_changed);
  EXPECT_NEAR(yaw.yaw_rad, 1.3, 1e-12);

  yaw = monitor.update_yaw(TimedYawSample{2.1, epoch + 330ms}, 0.0, epoch + 330ms);
  EXPECT_NEAR(yaw.yaw_rad, 1.4, 1e-12);
}

TEST(SafetyMonitorTest, RecoveryActionsAreRateLimitedAndSelfClearing)
{
  SafetyMonitor monitor{default_parameters()};
  const auto epoch = std::chrono::steady_clock::time_point{};
  std::array<DmMotorHealth, kMotorCount> health{};
  health[0].consecutive_missed_frames = 50U;
  health[1].has_feedback = true;
  health[1].error = MotorError::over_current;

  auto actions = monitor.recovery_actions(health, epoch);
  EXPECT_TRUE(actions.reenable[0]);
  EXPECT_TRUE(actions.clear_fault[1]);
  EXPECT_TRUE(actions.reenable[1]);

  actions = monitor.recovery_actions(health, epoch + 500ms);
  EXPECT_FALSE(actions.reenable[0]);
  EXPECT_FALSE(actions.clear_fault[1]);

  actions = monitor.recovery_actions(health, epoch + 1s);
  EXPECT_TRUE(actions.reenable[0]);
  EXPECT_TRUE(actions.clear_fault[1]);

  health[0].consecutive_missed_frames = 0U;
  health[1].error = MotorError::enabled;
  actions = monitor.recovery_actions(health, epoch + 1100ms);
  EXPECT_FALSE(actions.reenable[0]);
  EXPECT_FALSE(actions.clear_fault[1]);
  EXPECT_FALSE(actions.reenable[1]);
}

TEST(SafetyMonitorTest, BusSilentRequiresEveryMotorPastThreshold)
{
  SafetyMonitor monitor{default_parameters()};
  std::array<DmMotorHealth, kMotorCount> health{};
  for (auto & motor : health) {
    motor.consecutive_missed_frames = 50U;
  }
  EXPECT_TRUE(monitor.all_bus_silent(health));
  health[3].consecutive_missed_frames = 0U;
  EXPECT_FALSE(monitor.all_bus_silent(health));
}

TEST(SafetyMonitorTest, WheelLeastSquaresRecoversYawWithOneModuleMissing)
{
  constexpr double dx{0.1};
  constexpr double dy{-0.03};
  constexpr double dtheta{0.04};
  std::array<SwerveModulePosition, kSwerveModuleCount> previous{};
  std::array<SwerveModulePosition, kSwerveModuleCount> current{};
  for (std::size_t index{0U}; index < current.size(); ++index) {
    const double wheel_x{dx - dtheta * kLocations[index].y};
    const double wheel_y{dy + dtheta * kLocations[index].x};
    const double angle{std::atan2(wheel_y, wheel_x)};
    previous[index] = SwerveModulePosition{0.0, angle, true};
    current[index] = SwerveModulePosition{std::hypot(wheel_x, wheel_y), angle, true};
  }
  current[2].valid = false;

  const auto delta = wheel_chassis_delta_from_position_deltas(
    previous, current, kLocations);
  ASSERT_TRUE(delta.has_value());
  EXPECT_NEAR(delta->dx_m, dx, 1e-12);
  EXPECT_NEAR(delta->dy_m, dy, 1e-12);
  EXPECT_NEAR(delta->dtheta_rad, dtheta, 1e-12);
}

}  // namespace
}  // namespace dm_swerve_driver
