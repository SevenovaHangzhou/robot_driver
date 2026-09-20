#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "rt_arm_dynamics/arm_dynamics.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <iterator>
#include <limits>

using rt_arm_dynamics::ArmDynamics;
using rt_arm_dynamics::Config;
using rt_arm_dynamics::Result;

void begin_heap_probe() noexcept;
std::size_t end_heap_probe() noexcept;

namespace
{
Config config()
{
  std::ifstream file(ARM_FIXTURE);
  Config c;
  c.urdf = {std::istreambuf_iterator<char>(file),
    std::istreambuf_iterator<char>()};
  c.joints = {"right_joint1", "right_joint2"};
  c.locked_positions = {{"auxiliary_turn", 0.25}};
  c.input_inertia_g_mm2 = {1000.0, 2000.0};
  c.reduction_ratio = {10.0, 20.0};
  c.payload_frame = "tool";
  return c;
}
class DynamicsTest : public testing::Test
{
protected:
  ArmDynamics arm;
  std::string reason;
  void SetUp() override
  {
    ASSERT_EQ(arm.initialize(config(), reason), Result::kOk) << reason;
  }
};
} // namespace

TEST_F(DynamicsTest, DoublePendulumMatchesAnalyticalGravity) {
  Eigen::VectorXd q(2), out(2);
  for (double angle : {-1.0, 0.0, 0.8}) {
    q << angle, 0.3;
    ASSERT_EQ(arm.gravity(q, out), Result::kOk);
    const double second = -9.81 * 3.0 * 0.3 * std::cos(angle + 0.3);
    EXPECT_NEAR(
      out[0],
      -9.81 * (2.0 * 0.4 + 3.0 * 0.8) * std::cos(angle) + second,
      1e-10);
    EXPECT_NEAR(out[1], second, 1e-10);
  }
}

TEST_F(DynamicsTest, LockedJointProducesAnalyticalSinglePendulum) {
  auto c = config();
  c.joints = {"right_joint2"};
  c.locked_positions["right_joint1"] = 0.4;
  c.input_inertia_g_mm2 = {2000.0};
  c.reduction_ratio = {20.0};
  ASSERT_EQ(arm.initialize(c, reason), Result::kOk) << reason;
  Eigen::VectorXd q(1), out(1);
  q << 0.2;
  ASSERT_EQ(arm.gravity(q, out), Result::kOk);
  EXPECT_NEAR(out[0], -9.81 * 3.0 * 0.3 * std::cos(0.6), 1e-10);
}

TEST_F(DynamicsTest, KeepsRequestedJointOrderAndRenamesOnlyLinks) {
  auto c = config();
  std::reverse(c.joints.begin(), c.joints.end());
  std::reverse(c.input_inertia_g_mm2.begin(), c.input_inertia_g_mm2.end());
  std::reverse(c.reduction_ratio.begin(), c.reduction_ratio.end());
  c.payload_frame = "right_joint2"; // Original link name is accepted.
  ASSERT_EQ(arm.initialize(c, reason), Result::kOk) << reason;
  EXPECT_EQ(arm.joints(), c.joints);
  EXPECT_TRUE(arm.model().existJointName("right_joint2"));
  EXPECT_TRUE(arm.model().existFrame("right_link2"));
  Eigen::VectorXd q(2), out(2);
  q << 0.3, 0.0;
  ASSERT_EQ(arm.gravity(q, out), Result::kOk);
  EXPECT_NEAR(out[0], -9.81 * 0.9 * std::cos(0.3), 1e-10);
  EXPECT_NEAR(out[1], -9.81 * 3.2 + out[0], 1e-10);
}

TEST_F(DynamicsTest, RneaEqualsMassCoriolisGravityWithArmature) {
  const auto & m = arm.model();
  pinocchio::Data d(m);
  Eigen::VectorXd q(2), v(2), a(2);
  q << 0.4, -0.7;
  v << 0.2, 0.5;
  a << 1.1, -0.3;
  EXPECT_NEAR(m.armature[0], 1000.0 * 1e-9 * 100.0, 1e-15);
  EXPECT_NEAR(m.armature[1], 2000.0 * 1e-9 * 400.0, 1e-15);
  const Eigen::VectorXd torque = pinocchio::rnea(m, d, q, v, a);
  pinocchio::crba(m, d, q);
  const Eigen::MatrixXd mass = d.M.selfadjointView<Eigen::Upper>();
  const Eigen::MatrixXd coriolis = pinocchio::computeCoriolisMatrix(m, d, q, v);
  Eigen::VectorXd gravity(2);
  ASSERT_EQ(arm.gravity(q, gravity), Result::kOk);
  EXPECT_LT((torque - mass * a - coriolis * v - gravity).norm(), 1e-10);
}

TEST_F(DynamicsTest, PayloadDeltaMatchesNegativeGravityWrenchAtCom) {
  Eigen::VectorXd q(2), before(2), after(2);
  q << 0.2, -0.4;
  ASSERT_EQ(arm.gravity(q, before), Result::kOk);
  const Eigen::Vector3d com(0.13, 0.07, 0.04);
  const double mass = 1.7;
  // Separate reference model with a fixed frame at the payload COM.
  pinocchio::Model reference = arm.model();
  const auto tool = reference.getFrameId("tool");
  const auto parent = reference.frames[tool].parentJoint;
  const auto placement = reference.frames[tool].placement *
    pinocchio::SE3(Eigen::Matrix3d::Identity(), com);
  const auto point = reference.addFrame(
    pinocchio::Frame(
      "payload_com", parent, tool, placement, pinocchio::OP_FRAME));
  pinocchio::Data data(reference);
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, reference.nv);
  jacobian.setZero();
  pinocchio::computeJointJacobians(reference, data, q);
  pinocchio::updateFramePlacements(reference, data);
  pinocchio::getFrameJacobian(
    reference, data, point,
    pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  const Eigen::VectorXd expected =
    -jacobian.topRows<3>().transpose() * mass * config().gravity;
  ASSERT_EQ(arm.setPayload(mass, com, Eigen::Matrix3d::Zero()), Result::kOk);
  ASSERT_EQ(arm.gravity(q, after), Result::kOk);
  EXPECT_LT((after - before - expected).norm(), 1e-10);
  ASSERT_EQ(arm.setPayload(mass, com, Eigen::Matrix3d::Zero()), Result::kOk);
  Eigen::VectorXd repeated(2);
  arm.gravity(q, repeated);
  EXPECT_LT((repeated - after).norm(), 1e-12);
  ASSERT_EQ(
    arm.setPayload(0.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()),
    Result::kOk);
  arm.gravity(q, after);
  EXPECT_LT((after - before).norm(), 1e-12);
}

TEST_F(DynamicsTest, InvalidInputDoesNotCommitAnOutput) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(2),
    out = Eigen::VectorXd::Constant(2, 123.0);
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    q[0] = bad;
    EXPECT_EQ(arm.gravity(q, out), Result::kInvalidInput);
    EXPECT_EQ(out[0], 123.0);
  }
  q = Eigen::VectorXd::Zero(1);
  EXPECT_EQ(arm.gravity(q, out), Result::kInvalidInput);
  EXPECT_EQ(
    arm.setPayload(-1.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()),
    Result::kInvalidInput);
}

TEST_F(DynamicsTest, RejectsBadModelConfiguration) {
  for (int change = 0; change < 5; ++change) {
    auto c = config();
    if (change == 0) {
      c.joints[0] = "unknown";
    }
    if (change == 1) {
      c.joints[1] = c.joints[0];
    }
    if (change == 2) {
      c.payload_frame = "missing";
    }
    if (change == 3) {
      c.joints.pop_back();
      c.input_inertia_g_mm2.pop_back();
      c.reduction_ratio.pop_back();
    }
    if (change == 4) {
      c.gravity[0] = std::numeric_limits<double>::infinity();
    }
    EXPECT_NE(arm.initialize(c, reason), Result::kOk) << change;
    EXPECT_FALSE(reason.empty());
  }
}

TEST_F(DynamicsTest, FeedforwardRejectsUnsupportedMotionAndNonfiniteInput) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(2), v = q, a = q, out(2),
  expected(2);
  ASSERT_EQ(arm.gravity(q, expected), Result::kOk);
  ASSERT_EQ(arm.feedforward(q, v, a, out), Result::kOk);
  EXPECT_LT((out - expected).norm(), 1e-12);
  v[0] = 0.1;
  EXPECT_EQ(arm.feedforward(q, v, a, out), Result::kUnsupported);
  v[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(arm.feedforward(q, v, a, out), Result::kInvalidInput);
}

TEST_F(DynamicsTest, RuntimeCallsDoNotAllocateEigenMemory) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(2), out(2), zero = q;
  const Eigen::Vector3d com(0.1, 0.0, 0.0);
  const Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  begin_heap_probe();
  Eigen::internal::set_is_malloc_allowed(false);
  const auto payload_result = arm.setPayload(1.0, com, inertia);
  const auto gravity_result = arm.gravity(q, out);
  const auto ff_result = arm.feedforward(q, zero, zero, out);
  Eigen::internal::set_is_malloc_allowed(true);
  const auto allocations = end_heap_probe();
  EXPECT_EQ(payload_result, Result::kOk);
  EXPECT_EQ(gravity_result, Result::kOk);
  EXPECT_EQ(ff_result, Result::kOk);
  EXPECT_EQ(allocations, 0U);
}

TEST_F(DynamicsTest, ArmatureDoesNotAlterStaticGravity) {
  Eigen::VectorXd q = Eigen::VectorXd::Constant(2, 0.2), a(2), b(2);
  arm.gravity(q, a);
  auto c = config();
  c.input_inertia_g_mm2 = {0.0, 0.0};
  ASSERT_EQ(arm.initialize(c, reason), Result::kOk);
  arm.gravity(q, b);
  EXPECT_LT((a - b).norm(), 1e-12);
  c.gravity.setZero();
  ASSERT_EQ(arm.initialize(c, reason), Result::kOk);
  arm.gravity(q, b);
  EXPECT_DOUBLE_EQ(b.norm(), 0.0);
}

TEST_F(DynamicsTest, RenameCollisionIsRejected) {
  auto c = config();
  c.urdf.insert(c.urdf.find("</robot>"), "<link name=\"right_link1\"/>");
  EXPECT_EQ(arm.initialize(c, reason), Result::kInvalidModel);
}

TEST(DualSevenTest, ReducesEitherArmWithoutChangingSourceNames)
{
  std::ifstream file(ARM_SEVEN_FIXTURE);
  ASSERT_TRUE(file);
  const std::string source{std::istreambuf_iterator<char>(file),
    std::istreambuf_iterator<char>()};
  for (const auto & side : {std::string("left"), std::string("right")}) {
    Config config;
    config.urdf = source;
    config.payload_frame = side + "_tool";
    for (int joint = 1; joint <= 7; ++joint) {
      config.joints.push_back(side + "_joint" + std::to_string(joint));
      config.locked_positions[(side == "left" ? "right" : "left") +
        std::string("_joint") + std::to_string(joint)] = 0.0;
      config.input_inertia_g_mm2.push_back(1000.0);
      config.reduction_ratio.push_back(10.0);
    }
    ArmDynamics dynamics;
    std::string reason;
    ASSERT_EQ(dynamics.initialize(config, reason), Result::kOk) << reason;
    ASSERT_EQ(dynamics.size(), 7U);
    EXPECT_EQ(dynamics.joints(), config.joints);
    EXPECT_TRUE(dynamics.model().existJointName(side + "_joint7"));
    EXPECT_FALSE(
      dynamics.model().existJointName(
        (side == "left" ? "right" : "left") + std::string("_joint1")));
    Eigen::VectorXd q = Eigen::VectorXd::Zero(7), torque(7);
    begin_heap_probe();
    Eigen::internal::set_is_malloc_allowed(false);
    const auto calculation = dynamics.gravity(q, torque);
    Eigen::internal::set_is_malloc_allowed(true);
    const auto allocations = end_heap_probe();
    EXPECT_EQ(calculation, Result::kOk);
    EXPECT_EQ(allocations, 0U);
    EXPECT_TRUE(torque.allFinite());
    EXPECT_GT(torque.norm(), 0.0);
    EXPECT_NE(source.find("<link name=\"" + side + "_joint1\""), std::string::npos);
  }
}

TEST_F(DynamicsTest, ReportsTimingWithoutAnAcceptanceThreshold) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(2), out(2);
  constexpr int samples = 2000;
  double total = 0.0, maximum = 0.0;
  for (int i = 0; i < samples; ++i) {
    const auto start = std::chrono::steady_clock::now();
    const auto result = arm.gravity(q, out);
    const double elapsed = std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - start)
      .count();
    ASSERT_EQ(result, Result::kOk);
    total += elapsed;
    maximum = std::max(maximum, elapsed);
  }
  std::cout << "Offline synthetic 2-DOF gravity mean_us=" << total / samples
            << " max_us=" << maximum << " (not a realtime qualification)\n";
}
