#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>

namespace ctrl_gravity_feedforward
{

// URDF parsing, fixed-body aggregation and allocation occur only at construction.
// Gravity is expressed in base_link, and all positions use the URDF joint convention.
class GravityModel final
{
public:
  GravityModel(
    const std::string & urdf, const std::string & base_link, const std::string & tip_link,
    const std::array<double, 3> & gravity);
  GravityModel(const GravityModel &) = delete;
  GravityModel & operator=(const GravityModel &) = delete;

  // No allocation, locks or I/O. On failure the published torques are all zero.
  bool compute(const std::vector<double> & positions) noexcept;
  const std::vector<std::string> & joints() const noexcept { return joints_; }
  const std::vector<double> & torques() const noexcept { return output_; }

private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> solver_;
  KDL::JntArray positions_;
  KDL::JntArray torques_;
  std::vector<std::string> joints_;
  std::vector<double> output_;
};

}  // namespace ctrl_gravity_feedforward
