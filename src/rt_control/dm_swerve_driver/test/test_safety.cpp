#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>

#include "dm_swerve_driver/kinco_safety_monitor.hpp"

namespace dm_swerve_driver {
namespace {

using namespace std::chrono_literals;

[[nodiscard]] std::array<KincoAxisHealth, kKincoAxisCount> healthy_axes()
{
  std::array<KincoAxisHealth, kKincoAxisCount> axes{};
  for (auto & axis : axes) {
    axis.has_feedback = true;
    axis.condition = KincoAxisCondition::enabled;
  }
  return axes;
}

TEST(KincoSafetyMonitorTest, CommandWatchdogStopsAndRecovers)
{
  KincoSafetyMonitor monitor{default_parameters()};
  const auto epoch = std::chrono::steady_clock::time_point{};
  const ChassisSpeeds command{1.0, -0.2, 0.3};

  auto decision = monitor.command_for_cycle(command, epoch, epoch + 100ms);
  EXPECT_FALSE(decision.timed_out);
  EXPECT_DOUBLE_EQ(decision.command.vx_mps, 1.0);

  decision = monitor.command_for_cycle(command, epoch, epoch + 300ms);
  EXPECT_TRUE(decision.timed_out);
  EXPECT_DOUBLE_EQ(decision.command.vx_mps, 0.0);

  decision = monitor.command_for_cycle(command, epoch + 310ms, epoch + 320ms);
  EXPECT_FALSE(decision.timed_out);
}

TEST(KincoSafetyMonitorTest, ImuFallbackAndRecoveryKeepYawContinuous)
{
  KincoSafetyMonitor monitor{default_parameters()};
  const auto epoch = std::chrono::steady_clock::time_point{};

  auto yaw = monitor.update_yaw(TimedYawSample{1.0, epoch}, 0.0, epoch);
  EXPECT_FALSE(yaw.imu_fallback);
  yaw = monitor.update_yaw(std::nullopt, 0.1, epoch + 300ms);
  EXPECT_TRUE(yaw.imu_fallback);
  EXPECT_NEAR(yaw.yaw_rad, 1.1, 1e-12);
  yaw = monitor.update_yaw(TimedYawSample{2.0, epoch + 310ms}, 0.0, epoch + 310ms);
  EXPECT_FALSE(yaw.imu_fallback);
  EXPECT_NEAR(yaw.yaw_rad, 1.1, 1e-12);
}

TEST(KincoSafetyMonitorTest, RecoverableFaultUsesBoundedRetriesThenLatches)
{
  auto parameters = default_parameters();
  parameters.safety.auto_recovery_limit = 2U;
  KincoSafetyMonitor monitor{parameters};
  auto axes = healthy_axes();
  axes[1].condition = KincoAxisCondition::recoverable_fault;
  const auto epoch = std::chrono::steady_clock::time_point{};

  auto actions = monitor.recovery_actions(axes, epoch);
  EXPECT_TRUE(actions.clear_fault[1]);
  EXPECT_TRUE(actions.reenable[1]);
  EXPECT_TRUE(monitor.faulted());
  EXPECT_FALSE(monitor.fault_latched());

  actions = monitor.recovery_actions(axes, epoch + 1s);
  EXPECT_TRUE(actions.reenable[1]);
  actions = monitor.recovery_actions(axes, epoch + 2s);
  EXPECT_FALSE(actions.reenable[1]);
  EXPECT_TRUE(monitor.fault_latched());
}

TEST(KincoSafetyMonitorTest, LatchingAxisFaultNeverAutoClears)
{
  KincoSafetyMonitor monitor{default_parameters()};
  auto axes = healthy_axes();
  axes[2].condition = KincoAxisCondition::latching_fault;

  const auto actions = monitor.recovery_actions(axes, {});

  EXPECT_TRUE(monitor.fault_latched());
  EXPECT_TRUE(std::none_of(actions.reenable.begin(), actions.reenable.end(),
      [](bool value) {return value;}));
}

TEST(KincoSafetyMonitorTest, ManualClearRequiresAllAxesAndRecoveredTransport)
{
  KincoSafetyMonitor monitor{default_parameters()};
  auto axes = healthy_axes();
  std::array<bool, kKincoAxisCount> confirmed{};
  confirmed.fill(true);
  std::array<bool, kKincoAxisCount> received{};
  received.fill(true);

  monitor.mark_transport_failure();
  EXPECT_FALSE(monitor.complete_manual_clear(axes, confirmed));
  monitor.observe_feedback(received);
  EXPECT_TRUE(monitor.complete_manual_clear(axes, confirmed));

  EXPECT_TRUE(monitor.observe_steering_limit_violation(true));
  EXPECT_FALSE(monitor.complete_manual_clear(axes, confirmed));
  EXPECT_TRUE(monitor.observe_steering_limit_violation(false));
  EXPECT_TRUE(monitor.complete_manual_clear(axes, confirmed));
  EXPECT_FALSE(monitor.faulted());
}

}  // namespace
}  // namespace dm_swerve_driver
