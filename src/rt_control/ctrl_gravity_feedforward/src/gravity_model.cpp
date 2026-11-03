#include "ctrl_gravity_feedforward/gravity_model.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace ctrl_gravity_feedforward
{

GravityModel::GravityModel(
  const std::string & xml, const std::string & base, const std::string & tip,
  const std::array<double, 3> & gravity)
{
  for (const auto value : gravity) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("gravity must be finite and expressed in base_link");
    }
  }
  urdf::Model model;
  KDL::Tree tree;
  if (!model.initString(xml) || !kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::invalid_argument("invalid expanded robot_description");
  }
  auto ancestor = model.getLink(tip);
  while (ancestor && ancestor->name != base) {
    ancestor = ancestor->getParent();
  }
  if (
    !ancestor || base == tip || !tree.getChain(base, tip, chain_) || chain_.getNrOfJoints() == 0) {
    throw std::invalid_argument("tip_link must descend from base_link through moving joints");
  }

  // A tool can be a fixed sibling of the TCP. A plain getChain(base, TCP) would
  // omit its mass. Fold every off-chain fixed descendant into its owning segment.
  std::function<KDL::RigidBodyInertia(const std::string &)> fixed_subtree;
  fixed_subtree = [&](const std::string & name) {
    const auto & segment = tree.getSegment(name)->second.segment;
    auto inertia = segment.getInertia();
    for (const auto & child : model.getLink(name)->child_links) {
      if (child->parent_joint->type != urdf::Joint::FIXED) {
        throw std::invalid_argument("unmodelled moving branch: " + child->parent_joint->name);
      }
      const auto & child_segment = tree.getSegment(child->name)->second.segment;
      inertia = inertia + child_segment.pose(0.0) * fixed_subtree(child->name);
    }
    return inertia;
  };
  for (unsigned int index = 0; index < chain_.getNrOfSegments(); ++index) {
    auto & segment = chain_.getSegment(index);
    const auto link = model.getLink(segment.getName());
    if (link->parent_joint->mimic) {
      throw std::invalid_argument("mimic joint is unsupported: " + link->parent_joint->name);
    }
    auto inertia = segment.getInertia();
    const std::string next =
      index + 1 < chain_.getNrOfSegments() ? chain_.getSegment(index + 1).getName() : "";
    for (const auto & child : link->child_links) {
      if (child->name == next) {
        continue;
      }
      if (child->parent_joint->type != urdf::Joint::FIXED) {
        throw std::invalid_argument("unmodelled moving branch: " + child->parent_joint->name);
      }
      inertia = inertia +
                tree.getSegment(child->name)->second.segment.pose(0.0) * fixed_subtree(child->name);
    }
    if (!std::isfinite(inertia.getMass()) || inertia.getMass() < 0.0) {
      throw std::invalid_argument("invalid mass at " + segment.getName());
    }
    segment.setInertia(inertia);
    if (segment.getJoint().getType() != KDL::Joint::Fixed) {
      joints_.push_back(segment.getJoint().getName());
    }
  }
  positions_.resize(chain_.getNrOfJoints());
  torques_.resize(chain_.getNrOfJoints());
  output_.resize(joints_.size(), 0.0);
  solver_ =
    std::make_unique<KDL::ChainDynParam>(chain_, KDL::Vector(gravity[0], gravity[1], gravity[2]));
}

bool GravityModel::compute(const std::vector<double> & positions) noexcept
{
  std::fill(output_.begin(), output_.end(), 0.0);
  if (positions.size() != joints_.size()) {
    return false;
  }
  for (unsigned int index = 0; index < positions_.rows(); ++index) {
    if (!std::isfinite(positions[index])) {
      return false;
    }
    positions_(index) = positions[index];
  }
  if (solver_->JntToGravity(positions_, torques_) != 0) {
    return false;
  }
  for (unsigned int index = 0; index < torques_.rows(); ++index) {
    if (!std::isfinite(torques_(index))) {
      return false;
    }
  }
  for (unsigned int index = 0; index < torques_.rows(); ++index) {
    output_[index] = torques_(index);
  }
  return true;
}

}  // namespace ctrl_gravity_feedforward
