#include "rt_arm_dynamics/arm_dynamics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
namespace rt_arm_dynamics
{
Result ArmDynamics::gravity(
  Eigen::Ref<const Eigen::VectorXd> q,
  Eigen::Ref<Eigen::VectorXd> out) noexcept
{
  if (!data_) {
    return Result::kNotInitialized;
  }
  if (q.size() != model_.nv || out.size() != model_.nv || !q.allFinite()) {
    return Result::kInvalidInput;
  }
  for (std::size_t axis = 0U; axis < indices_.size(); ++axis) {
    q_[indices_[axis]] = q[static_cast<Eigen::Index>(axis)];
  }
  const auto & torque = pinocchio::rnea(model_, *data_, q_, zero_, zero_);
  if (!torque.allFinite()) {
    return Result::kNonfiniteResult;
  }
  for (std::size_t axis = 0U; axis < indices_.size(); ++axis) {
    out[static_cast<Eigen::Index>(axis)] = torque[indices_[axis]];
  }
  return Result::kOk;
}
Result ArmDynamics::feedforward(
  Eigen::Ref<const Eigen::VectorXd> q,
  Eigen::Ref<const Eigen::VectorXd> v,
  Eigen::Ref<const Eigen::VectorXd> a,
  Eigen::Ref<Eigen::VectorXd> out) noexcept
{
  if (!data_) {
    return Result::kNotInitialized;
  }
  if (q.size() != model_.nv || v.size() != model_.nv || a.size() != model_.nv ||
    out.size() != model_.nv || !q.allFinite() || !v.allFinite() ||
    !a.allFinite())
  {
    return Result::kInvalidInput;
  }
  if (!v.isZero(0.0) || !a.isZero(0.0)) {
    return Result::kUnsupported;
  }
  return gravity(q, out);
}
Result ArmDynamics::setPayload(
  double mass, const Eigen::Vector3d & com,
  const Eigen::Matrix3d & inertia) noexcept
{
  if (!data_) {
    return Result::kNotInitialized;
  }
  if (!std::isfinite(mass) || mass < 0.0 || !com.allFinite() ||
    !inertia.allFinite() || !inertia.isApprox(inertia.transpose(), 1e-12))
  {
    return Result::kInvalidInput;
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
    inertia, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success ||
    solver.eigenvalues().minCoeff() < 0.0 ||
    (mass == 0.0 && !inertia.isZero(0.0)))
  {
    return Result::kInvalidInput;
  }
  const pinocchio::Inertia payload(mass, com, inertia);
  const auto combined = base_inertia_ + payload_placement_.act(payload);
  if (!std::isfinite(combined.mass()) || !combined.lever().allFinite() ||
    !combined.inertia().matrix().allFinite())
  {
    return Result::kNonfiniteResult;
  }
  model_.inertias[payload_joint_] = combined;
  return Result::kOk;
}
} // namespace rt_arm_dynamics
