#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include "ctrl_swerve/navigation_feedback.hpp"
#include "ctrl_swerve/steering_velocity_observer.hpp"
#include "navigation_feedback_test_support.hpp"

namespace ctrl_swerve
{
namespace
{
using W = ExecutionStateWire;

TEST(NavigationWire, EncodesActualCoreStateWithDeployedOffsets)
{
  const auto config = test::feedback_config();
  ControlCore core(config);
  auto feedback = test::stopped_feedback();
  feedback[0].steering_angle = 0.01;  // Monitor differs from the actual control angle.
  ASSERT_TRUE(core.activate(feedback));
  core.update(feedback, {0.0, 0.3, 0.0}, true, 0.004);
  const auto & snapshot = core.execution_snapshot();
  const auto data = encode_execution_state(snapshot, 123.25, 17, true, true, snapshot.status);
  ASSERT_EQ(data.size(), 91U);
  EXPECT_DOUBLE_EQ(data[0], 1.0);
  EXPECT_DOUBLE_EQ(data[1], 123.25);
  EXPECT_DOUBLE_EQ(data[2], 17.0);
  ASSERT_DOUBLE_EQ(data[3], 1.0);
  for (const double value : data) {
    EXPECT_TRUE(std::isfinite(value));
  }
  EXPECT_DOUBLE_EQ(data[7], snapshot.translation_branch);
  EXPECT_DOUBLE_EQ(data[8], snapshot.body_reference.vx_mps);
  EXPECT_DOUBLE_EQ(data[10], snapshot.body_reference.omega_radps);
  EXPECT_DOUBLE_EQ(data[13], snapshot.body_acceleration.omega_radps);
  EXPECT_DOUBLE_EQ(data[24], snapshot.gate);
  EXPECT_DOUBLE_EQ(data[25], snapshot.release);
  EXPECT_DOUBLE_EQ(data[26], snapshot.flip_hysteresis);
  for (std::size_t i = 0; i < 4; ++i) {
    const auto base = 27 + 16 * i;
    const auto & module = snapshot.modules[i];
    EXPECT_DOUBLE_EQ(data[base], static_cast<double>(i + 1));
    EXPECT_DOUBLE_EQ(data[base + 1], module.steering_actual);
    EXPECT_DOUBLE_EQ(data[base + 2], module.wheel_actual);
    EXPECT_DOUBLE_EQ(data[base + 3], module.steering_reference);
    EXPECT_DOUBLE_EQ(data[base + 4], module.steering_rate);
    EXPECT_DOUBLE_EQ(data[base + 5], module.steering_acceleration);
    EXPECT_DOUBLE_EQ(data[base + 6], module.wheel_reference);
    EXPECT_DOUBLE_EQ(data[base + 7], module.reversed ? 1.0 : 0.0);
    EXPECT_DOUBLE_EQ(data[base + 8], module.holding ? 1.0 : 0.0);
    EXPECT_DOUBLE_EQ(data[base + 9], module.steering_target);
    EXPECT_DOUBLE_EQ(data[base + 10], module.steering_error);
    EXPECT_DOUBLE_EQ(data[base + 14], config.locations[i].x);
    EXPECT_DOUBLE_EQ(data[base + 15], config.locations[i].y);
  }
  EXPECT_GT(data[27 + 9], data[27 + 3]);
}

TEST(NavigationWire, UnavailableFieldsCannotAdvertiseReady)
{
  ControlCore core(test::feedback_config());
  const auto feedback = test::stopped_feedback();
  ASSERT_TRUE(core.activate(feedback));
  core.update(feedback, {}, true, 0.004);
  auto snapshot = core.execution_snapshot();
  snapshot.modules[2].steering_rate = std::numeric_limits<double>::infinity();
  auto data = encode_execution_state(snapshot, 10.0, 1, true, true, ControlStatus::running);
  EXPECT_DOUBLE_EQ(data[W::Ready], 0.0);
  EXPECT_TRUE(std::isnan(data[W::Modules + 2 * W::Stride + W::ReferenceRate]));
  data = encode_execution_state(
    core.execution_snapshot(), 10.0, 2, false, false, ControlStatus::inactive);
  EXPECT_DOUBLE_EQ(data[W::Ready], 0.0);
  EXPECT_DOUBLE_EQ(data[W::Status], 0.0);
  EXPECT_TRUE(std::isnan(data[W::Modules + W::Angle]));
}

class SteeringObservationTest : public testing::Test
{
protected:
  SteeringVelocityObserver observer;
  double sample(double angle, double age_ms, double now, bool valid = true)
  {
    return observer.observe(angle, age_ms, now, valid, 0.02, -3.2, 3.2, 5.0, 0.002);
  }
  void establish(double angle = 0.2)
  {
    ASSERT_TRUE(std::isnan(sample(angle, 0.5, 10.0)));
    ASSERT_TRUE(std::isnan(sample(angle, 3.5, 10.003)));
    ASSERT_NEAR(sample(angle + 0.004, 0.5, 10.004), 1.0, 1e-9);
  }
};

TEST_F(SteeringObservationTest, RequiresTwoReceiveQualifiedFrames)
{
  establish();
}

TEST_F(SteeringObservationTest, RepeatedOldPositionDoesNotDifferentiateToZero)
{
  establish();
  EXPECT_NEAR(sample(0.204, 1.5, 10.005), 1.0, 1e-9);
  EXPECT_NEAR(sample(0.204, 2.5, 10.006), 1.0, 1e-9);
  EXPECT_NEAR(sample(0.204, 3.5, 10.007), 1.0, 1e-9);
  EXPECT_NEAR(sample(0.204, 0.5, 10.008), 0.0, 1e-9);  // A real new stationary frame.
}

TEST_F(SteeringObservationTest, FrozenZeroAgeCannotManufactureStationaryEvidence)
{
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(std::isnan(sample(0.2, 0.0, 10.0 + 0.001 * i)));
  }
}

TEST_F(SteeringObservationTest, StalenessAndInvalidationRebuildHistory)
{
  establish();
  EXPECT_TRUE(std::isnan(sample(0.204, 21.0, 10.025)));
  EXPECT_TRUE(std::isnan(sample(0.204, 0.5, 10.026)));
  EXPECT_TRUE(std::isnan(sample(0.204, 3.5, 10.029)));
  EXPECT_DOUBLE_EQ(sample(0.204, 0.5, 10.030), 0.0);
  EXPECT_TRUE(std::isnan(sample(0.204, 0.0, 10.031, false)));
  EXPECT_TRUE(std::isnan(sample(0.204, 0.5, 10.032)));
  EXPECT_TRUE(std::isnan(sample(std::numeric_limits<double>::quiet_NaN(), 0.0, 10.033)));
  EXPECT_TRUE(std::isnan(sample(0.204, 0.5, 10.034)));
}

TEST_F(SteeringObservationTest, LifecycleResetDoesNotReuseAnOldRate)
{
  establish();
  observer.reset();
  EXPECT_TRUE(std::isnan(sample(0.204, 0.5, 10.005)));
  EXPECT_TRUE(std::isnan(sample(0.204, 3.5, 10.008)));
  EXPECT_DOUBLE_EQ(sample(0.204, 0.5, 10.009), 0.0);
}

TEST_F(SteeringObservationTest, ClockRegressionAndUpdateGapRebuildHistory)
{
  establish();
  EXPECT_TRUE(std::isnan(sample(0.204, 0.0, 9.0)));
  EXPECT_TRUE(std::isnan(sample(0.204, 1.0, 9.001)));
  EXPECT_DOUBLE_EQ(sample(0.204, 0.0, 9.004), 0.0);
  EXPECT_TRUE(std::isnan(sample(0.204, 0.0, 10.0)));
}

TEST_F(SteeringObservationTest, MechanicalAnglesNeverWrapAcrossAReset)
{
  EXPECT_TRUE(std::isnan(sample(3.1, 0.5, 10.0)));
  EXPECT_TRUE(std::isnan(sample(3.1, 18.5, 10.018)));
  EXPECT_TRUE(std::isnan(sample(-3.1, 0.5, 10.019)));
  EXPECT_TRUE(std::isnan(sample(-3.1, 1.5, 10.020)));
  EXPECT_TRUE(std::isnan(sample(3.3, 0.5, 10.021)));  // Outside bounded travel.
}
}  // namespace
}  // namespace ctrl_swerve
