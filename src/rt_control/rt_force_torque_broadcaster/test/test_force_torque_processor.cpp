#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "rt_force_torque_broadcaster/force_torque_processor.hpp"

namespace rt_force_torque_broadcaster
{
namespace
{

ForceTorqueProcessorConfig x503_config(bool calibration_valid = true)
{
  return ForceTorqueProcessorConfig{
    {0.1, 0.1, 0.1, 0.001, 0.001, 0.001},
    AuxiliaryPolicy::kAllExactIntegersInRange,
    6U,
    -999999,
    999999,
    calibration_valid};
}

rt_control_semantic_components::ForceTorqueSample x503_sample()
{
  return {
    {100.0, 200.0, 300.0, 1000.0, 2000.0, 3000.0},
    {0.0, 1.0, 2.0, 3.0, 4.0, 5.0},
    6U};
}

TEST(ForceTorqueProcessorTest, ConvertsValidatedX503DataOnlyAfterOperation)
{
  ForceTorqueProcessor processor{x503_config()};

  const auto before_op = processor.process(x503_sample(), 1.0, 2.0);
  EXPECT_TRUE(before_op.raw_valid);
  EXPECT_FALSE(before_op.wrench_valid);

  const auto in_op = processor.process(x503_sample(), 1.0, 8.0);
  ASSERT_TRUE(in_op.raw_valid);
  ASSERT_TRUE(in_op.wrench_valid);
  EXPECT_EQ(
    in_op.raw_values,
    (std::array<std::int32_t, 6>{100, 200, 300, 1000, 2000, 3000}));
  EXPECT_EQ(
    in_op.wrench_values,
    (std::array<double, 6>{10.0, 20.0, 30.0, 1.0, 2.0, 3.0}));
}

TEST(ForceTorqueProcessorTest, InvalidCalibrationStillAllowsRawButNeverWrench)
{
  ForceTorqueProcessor processor{x503_config(false)};

  const auto outcome = processor.process(x503_sample(), 1.0, 8.0);
  EXPECT_TRUE(outcome.raw_valid);
  EXPECT_FALSE(outcome.wrench_valid);
  EXPECT_FALSE(outcome.runtime_invalidated);
}

TEST(ForceTorqueProcessorTest, AuxiliaryRangeRejectsOnlyEngineeringWrench)
{
  ForceTorqueProcessor processor{x503_config()};
  auto sample = x503_sample();
  sample.auxiliary[4] = 1000000.0;

  const auto outcome = processor.process(sample, 1.0, 8.0);
  EXPECT_TRUE(outcome.raw_valid);
  EXPECT_FALSE(outcome.wrench_valid);
}

TEST(ForceTorqueProcessorTest, LinkLossAfterOperationLatchesUntilNewStartup)
{
  ForceTorqueProcessor processor{x503_config()};
  ASSERT_TRUE(processor.process(x503_sample(), 1.0, 8.0).wrench_valid);

  const auto lost = processor.process(x503_sample(), 0.0, 8.0);
  EXPECT_TRUE(lost.raw_valid);
  EXPECT_FALSE(lost.wrench_valid);
  EXPECT_TRUE(lost.runtime_invalidated);
  EXPECT_TRUE(lost.invalidated_this_cycle);

  const auto recovered = processor.process(x503_sample(), 1.0, 8.0);
  EXPECT_FALSE(recovered.wrench_valid);
  EXPECT_TRUE(recovered.runtime_invalidated);
  EXPECT_FALSE(recovered.invalidated_this_cycle);
}

TEST(ForceTorqueProcessorTest, NonintegralOrNonfiniteRawFrameIsRejected)
{
  for (const double invalid : {
        0.5,
        static_cast<double>(std::numeric_limits<std::int32_t>::max()) + 1.0,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()})
  {
    ForceTorqueProcessor processor{x503_config()};
    auto sample = x503_sample();
    sample.values[2] = invalid;
    const auto outcome = processor.process(sample, 1.0, 8.0);
    EXPECT_FALSE(outcome.raw_valid) << invalid;
    EXPECT_FALSE(outcome.wrench_valid) << invalid;
  }
}

TEST(ForceTorqueProcessorTest, NoAuxiliaryPolicySupportsFixedScaleSensors)
{
  const ForceTorqueProcessorConfig config{
    {0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001},
    AuxiliaryPolicy::kNone,
    3U,
    0,
    0,
    true};
  ForceTorqueProcessor processor{config};
  auto sample = x503_sample();
  sample.auxiliary_count = 3U;
  sample.auxiliary = {
    4294967295.0, 4294967295.0, -100.0, 0.0, 0.0, 0.0};

  const auto outcome = processor.process(sample, 1.0, 8.0);
  EXPECT_TRUE(outcome.raw_valid);
  EXPECT_TRUE(outcome.wrench_valid);
  EXPECT_DOUBLE_EQ(outcome.wrench_values[0], 0.01);
}

TEST(ForceTorqueProcessorTest, RejectsUnsafeConfiguration)
{
  auto config = x503_config();
  config.scale_factors[1] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(static_cast<void>(ForceTorqueProcessor{config}), std::invalid_argument);

  config = x503_config();
  config.auxiliary_count = 7U;
  EXPECT_THROW(static_cast<void>(ForceTorqueProcessor{config}), std::invalid_argument);

  config = x503_config();
  config.minimum_auxiliary_value = 10;
  config.maximum_auxiliary_value = 9;
  EXPECT_THROW(static_cast<void>(ForceTorqueProcessor{config}), std::invalid_argument);
}

}  // namespace
}  // namespace rt_force_torque_broadcaster
