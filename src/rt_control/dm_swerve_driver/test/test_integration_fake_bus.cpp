#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>

#include "dm_swerve_driver/control_loop.hpp"
#include "fake_can_transport.hpp"

namespace dm_swerve_driver {
namespace {

using namespace std::chrono_literals;

TEST(FakeBusIntegrationTest, StraightDriveConvergesWithoutLateralDrift)
{
  auto parameters = default_parameters();
  parameters.control.cmd_vel_timeout_s = 0.05;
  ControlLoop loop{
    parameters, std::make_unique<test::FakeCanTransport>()};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));

  for (int cycle{1}; cycle <= 200; ++cycle) {
    const auto now = epoch + cycle * 10ms;
    loop.submit_command(ChassisSpeeds{0.5, 0.0, 0.0}, now);
    loop.submit_imu_yaw(0.0, now);
    ASSERT_TRUE(loop.step(now));
  }

  const Pose2d pose{loop.status().pose};
  EXPECT_GT(pose.x_m, 0.5);
  EXPECT_NEAR(pose.y_m, 0.0, 0.02);
  EXPECT_NEAR(pose.heading_rad, 0.0, 1e-9);
}

TEST(FakeBusIntegrationTest, CircularDriveConvergesInGyroHeadingDirection)
{
  auto parameters = default_parameters();
  parameters.control.cmd_vel_timeout_s = 0.05;
  ControlLoop loop{
    parameters, std::make_unique<test::FakeCanTransport>()};
  const auto epoch = std::chrono::steady_clock::time_point{};
  ASSERT_TRUE(loop.initialize(epoch));

  constexpr double omega_radps{0.5};
  for (int cycle{1}; cycle <= 200; ++cycle) {
    const auto now = epoch + cycle * 10ms;
    loop.submit_command(ChassisSpeeds{0.5, 0.0, omega_radps}, now);
    loop.submit_imu_yaw(omega_radps * static_cast<double>(cycle) * 0.01, now);
    ASSERT_TRUE(loop.step(now));
  }

  const Pose2d pose{loop.status().pose};
  EXPECT_NEAR(pose.heading_rad, 0.995, 1e-9);
  EXPECT_GT(pose.x_m, 0.3);
  EXPECT_GT(pose.y_m, 0.1);
  EXPECT_LT(pose.y_m, pose.x_m);
}

}  // namespace
}  // namespace dm_swerve_driver
