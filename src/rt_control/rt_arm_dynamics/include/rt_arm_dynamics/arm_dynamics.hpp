#ifndef RT_ARM_DYNAMICS__ARM_DYNAMICS_HPP_
#define RT_ARM_DYNAMICS__ARM_DYNAMICS_HPP_

#include "rt_arm_dynamics/pinocchio_compat.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rt_arm_dynamics
{
enum class Result
{
  kOk,
  kNotInitialized,
  kInvalidInput,
  kInvalidModel,
  kUnsupported,
  kNonfiniteResult
};

struct Config
{
  std::string urdf;
  std::vector<std::string> joints;
  std::map<std::string, double> locked_positions;
  Eigen::Vector3d gravity{0.0, 0.0, -9.81};
  std::vector<double> input_inertia_g_mm2;
  std::vector<double> reduction_ratio;
  std::string payload_frame;
};

// Single-owner object. Initialize outside the realtime loop. Runtime calls and
// payload changes must be serialized by that loop, never by a callback mutex.
class ArmDynamics final
{
public:
  Result initialize(const Config & config, std::string & reason);
  Result gravity(
    Eigen::Ref<const Eigen::VectorXd> q,
    Eigen::Ref<Eigen::VectorXd> out) noexcept;
  Result feedforward(
    Eigen::Ref<const Eigen::VectorXd> q,
    Eigen::Ref<const Eigen::VectorXd> v,
    Eigen::Ref<const Eigen::VectorXd> a,
    Eigen::Ref<Eigen::VectorXd> out) noexcept;
  Result setPayload(
    double mass, const Eigen::Vector3d & com,
    const Eigen::Matrix3d & inertia) noexcept;
  std::size_t size() const noexcept {return indices_.size();}
  // Non-RT inspection for model validation and independent dynamics tests.
  const pinocchio::Model & model() const noexcept {return model_;}
  const std::vector<std::string> & joints() const noexcept {return joints_;}

private:
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::vector<Eigen::Index> indices_;
  std::vector<std::string> joints_;
  Eigen::VectorXd q_, zero_;
  pinocchio::JointIndex payload_joint_{0U};
  pinocchio::SE3 payload_placement_;
  pinocchio::Inertia base_inertia_;
};
} // namespace rt_arm_dynamics
#endif // RT_ARM_DYNAMICS__ARM_DYNAMICS_HPP_
