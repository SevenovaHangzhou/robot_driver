#include <gtest/gtest.h>

#include <chrono>

#include "dm_swerve_driver/steering_rezero.hpp"
#include "fake_can_transport.hpp"

namespace dm_swerve_driver {
namespace {

TEST(SteeringRezeroTest, ConfirmsSaveRepliesAndIndependentZeroReadback)
{
  test::FakeCanTransport transport;
  transport.open();

  const auto result = perform_steering_rezero(default_parameters(), transport);

  EXPECT_TRUE(result.success());
  ASSERT_EQ(transport.batches().size(), 2U);
  EXPECT_EQ(transport.batches()[0].size(), kSwerveModuleCount);
  EXPECT_EQ(transport.batches()[1].size(), kSwerveModuleCount);
}

TEST(SteeringRezeroTest, IdentifiesMotorWhoseSaveReplyIsMissing)
{
  test::FakeCanTransport transport;
  transport.open();
  transport.set_dropped_mst_id(0x12U);

  const auto result = perform_steering_rezero(default_parameters(), transport);

  ASSERT_FALSE(result.success());
  ASSERT_EQ(result.failures.size(), 1U);
  EXPECT_EQ(result.failures[0].esc_id, 2U);
  EXPECT_EQ(
    result.failures[0].reason,
    SteeringRezeroFailureReason::save_zero_acknowledgement_missing);
}

TEST(SteeringRezeroTest, IdentifiesMotorWhoseVerifiedPositionIsNotZero)
{
  auto parameters = default_parameters();
  parameters.steering.rezero_tolerance_rad = 0.05;
  test::FakeCanTransport transport;
  transport.open();
  transport.set_position_override(0x13U, 0.2);

  const auto result = perform_steering_rezero(parameters, transport);

  ASSERT_FALSE(result.success());
  ASSERT_EQ(result.failures.size(), 1U);
  EXPECT_EQ(result.failures[0].esc_id, 3U);
  EXPECT_EQ(
    result.failures[0].reason,
    SteeringRezeroFailureReason::position_out_of_tolerance);
}

}  // namespace
}  // namespace dm_swerve_driver
