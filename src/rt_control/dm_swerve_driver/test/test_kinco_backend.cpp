#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include "dm_swerve_driver/kinco_backend.hpp"
#include "dm_swerve_driver/kinco_units.hpp"
#include "fake_kinco_ethercat_bus.hpp"

namespace dm_swerve_driver {
namespace {

[[nodiscard]] KincoSwerveHardwareConfig hardware_config()
{
  KincoSwerveHardwareConfig config;
  config.steering_encoder_resolution.fill(10000U);
  config.drive_encoder_resolution.fill(10000U);
  config.steering_gear_ratio = 2.0;
  config.drive_gear_ratio = 3.0;
  config.wheel_radius_m = 0.1;
  config.startup_cycle_limit = 10U;
  config.position_velocity_feedforward_raw = 100U;
  config.position_acceleration_feedforward = 32767U;
  config.fault_reaction_option_code = 1;
  return config;
}

TEST(KincoBackendTest, StartupPreloadsActualPositionBeforeCoordinatedEnable)
{
  auto bus = std::make_unique<test::FakeKincoEthercatBus>();
  auto * observer = bus.get();
  KincoSwerveHardware hardware{hardware_config(), std::move(bus)};

  ASSERT_TRUE(hardware.initialize());
  EXPECT_TRUE(hardware.initialized());
  EXPECT_TRUE(observer->active());
  ASSERT_GE(observer->batches().size(), 4U);

  std::array<std::int32_t, kSwerveModuleCount> preload{};
  for (std::size_t index{0U}; index < preload.size(); ++index) {
    preload[index] = static_cast<std::int32_t>(1000U * (index + 1U));
  }
  for (const auto & batch : observer->batches()) {
    for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
      if (batch[index].control_word == 0x000FU) {
        EXPECT_EQ(batch[index].target_position, preload[index]);
      }
    }
  }
  for (const auto & axis : observer->batches().back()) {
    EXPECT_EQ(axis.control_word, 0x000FU);
  }
}

TEST(KincoBackendTest, StartupWritesOnlyExplicitPreopParameters)
{
  auto config = hardware_config();
  auto bus = std::make_unique<test::FakeKincoEthercatBus>();
  auto * observer = bus.get();
  KincoSwerveHardware hardware{config, std::move(bus)};

  ASSERT_TRUE(hardware.initialize());
  const auto & writes = observer->preop_writes();
  EXPECT_EQ(writes.size(), 16U);
  EXPECT_EQ(std::count_if(writes.begin(), writes.end(), [](const KincoSdoWrite & write) {
      return write.index == 0x60FBU && write.subindex == 2U && write.raw_value == 100U;
    }), 4);
  EXPECT_EQ(std::count_if(writes.begin(), writes.end(), [](const KincoSdoWrite & write) {
      return write.index == 0x60FBU && write.subindex == 3U && write.raw_value == 32767U;
    }), 4);
  EXPECT_EQ(std::count_if(writes.begin(), writes.end(), [](const KincoSdoWrite & write) {
      return write.index == 0x605EU && write.raw_value == 1U;
    }), 8);
}

TEST(KincoBackendTest, StrictStartupRejectsUnhealthyDomainOrMissingAxis)
{
  auto bus = std::make_unique<test::FakeKincoEthercatBus>();
  auto * observer = bus.get();
  observer->set_domain_status(EthercatDomainStatus{15U, 16U, true, true});
  KincoSwerveHardware unhealthy{hardware_config(), std::move(bus)};
  EXPECT_FALSE(unhealthy.initialize());
  EXPECT_FALSE(observer->is_open());

  auto missing_bus = std::make_unique<test::FakeKincoEthercatBus>();
  auto * missing_observer = missing_bus.get();
  missing_observer->set_axis_online(6U, false);
  KincoSwerveHardware missing{hardware_config(), std::move(missing_bus)};
  EXPECT_FALSE(missing.initialize());
  EXPECT_FALSE(missing_observer->is_open());
}

TEST(KincoBackendTest, OneHardwareCycleUsesOneAtomicEightAxisExchange)
{
  auto bus = std::make_unique<test::FakeKincoEthercatBus>();
  auto * observer = bus.get();
  const auto config = hardware_config();
  KincoSwerveHardware hardware{config, std::move(bus)};
  ASSERT_TRUE(hardware.initialize());
  const std::size_t batches_before{observer->batches().size()};
  std::array<KincoModuleTarget, kSwerveModuleCount> targets{};
  for (std::size_t index{0U}; index < targets.size(); ++index) {
    targets[index] = KincoModuleTarget{
      0.1 * static_cast<double>(index + 1U),
      0.2 * static_cast<double>(index + 1U),
      true};
  }

  const auto result = hardware.exchange(targets);

  EXPECT_TRUE(result.valid);
  ASSERT_EQ(observer->batches().size(), batches_before + 1U);
  const auto & batch = observer->batches().back();
  for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
    EXPECT_EQ(batch[index].mode, KincoOperationMode::cyclic_synchronous_position);
    EXPECT_EQ(batch[index].target_velocity, 0);
    EXPECT_EQ(batch[index + kSwerveModuleCount].mode,
      KincoOperationMode::cyclic_synchronous_velocity);
    const double motor_velocity = targets[index].wheel_speed_mps /
      config.wheel_radius_m * config.drive_gear_ratio;
    EXPECT_EQ(
      batch[index + kSwerveModuleCount].target_velocity,
      kinco_velocity_from_radians_per_second(
        motor_velocity, config.drive_encoder_resolution[index]));
    EXPECT_NEAR(result.modules[index].motor_steering_angle_rad,
      targets[index].steering_angle_rad, 1e-3);
    EXPECT_NEAR(result.modules[index].wheel_speed_mps,
      targets[index].wheel_speed_mps, 1e-3);
  }
}

TEST(KincoBackendTest, RejectsOutOfBoundsSteeringBeforeBusExchange)
{
  auto bus = std::make_unique<test::FakeKincoEthercatBus>();
  auto * observer = bus.get();
  KincoSwerveHardware hardware{hardware_config(), std::move(bus)};
  ASSERT_TRUE(hardware.initialize());
  const std::size_t batches_before{observer->batches().size()};
  std::array<KincoModuleTarget, kSwerveModuleCount> targets{};
  targets[0].steering_angle_rad = kPi + 0.01;

  EXPECT_THROW(static_cast<void>(hardware.exchange(targets)), std::out_of_range);
  EXPECT_EQ(observer->batches().size(), batches_before);
}

TEST(KincoBackendTest, FeedbackFaultIsClassifiedAndInvalidatesCycle)
{
  auto bus = std::make_unique<test::FakeKincoEthercatBus>();
  auto * observer = bus.get();
  KincoSwerveHardware hardware{hardware_config(), std::move(bus)};
  ASSERT_TRUE(hardware.initialize());
  observer->set_next_error(5U, 1U << 7U);

  const auto result = hardware.exchange({});

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.axis_faults[5].disposition, FaultDisposition::latch);
  EXPECT_TRUE(result.axis_faults[5].over_current);
}

}  // namespace
}  // namespace dm_swerve_driver
