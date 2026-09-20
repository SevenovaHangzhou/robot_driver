#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "rt_arm_dynamics/arm_dynamics.hpp"
#include <cmath>
#include <regex>
#include <set>
#include <stdexcept>
#include <tinyxml2.h>

namespace rt_arm_dynamics
{
namespace
{
std::string sanitized_urdf(
  const std::string & text,
  std::map<std::string, std::string> & aliases)
{
  tinyxml2::XMLDocument xml;
  if (xml.Parse(text.c_str(), text.size()) != tinyxml2::XML_SUCCESS) {
    throw std::invalid_argument("invalid URDF XML");
  }
  auto * robot = xml.FirstChildElement("robot");
  if (robot == nullptr) {
    throw std::invalid_argument("missing robot element");
  }
  std::set<std::string> names;
  for (auto * link = robot->FirstChildElement("link"); link != nullptr;
    link = link->NextSiblingElement("link"))
  {
    const char * name = link->Attribute("name");
    if (name == nullptr || !names.insert(name).second) {
      throw std::invalid_argument("missing or duplicate link name");
    }
  }
  // Temporary model-reader workaround only. No robot_description file or
  // public frame/joint identity is changed. Remove after upstream names are
  // fixed.
  const std::regex pattern("^(left|right)_joint([1-7])$");
  for (auto * link = robot->FirstChildElement("link"); link != nullptr;
    link = link->NextSiblingElement("link"))
  {
    const std::string name = link->Attribute("name");
    std::smatch match;
    if (std::regex_match(name, match, pattern)) {
      const std::string replacement = match[1].str() + "_link" + match[2].str();
      if (names.count(replacement) != 0U) {
        throw std::invalid_argument("link rename collision");
      }
      aliases.emplace(name, replacement);
      link->SetAttribute("name", replacement.c_str());
    }
  }
  for (auto * joint = robot->FirstChildElement("joint"); joint != nullptr;
    joint = joint->NextSiblingElement("joint"))
  {
    for (const char * tag : {"parent", "child"}) {
      auto * element = joint->FirstChildElement(tag);
      if (element == nullptr || element->Attribute("link") == nullptr) {
        throw std::invalid_argument("joint is missing parent/child link");
      }
      const auto alias = aliases.find(element->Attribute("link"));
      if (alias != aliases.end()) {
        element->SetAttribute("link", alias->second.c_str());
      }
    }
  }
  tinyxml2::XMLPrinter printer;
  xml.Print(&printer);
  return printer.CStr();
}
} // namespace

Result ArmDynamics::initialize(const Config & config, std::string & reason)
{
  data_.reset();
  indices_.clear();
  joints_.clear();
  reason.clear();
  try {
    const auto count = config.joints.size();
    if (count == 0U || !config.gravity.allFinite() ||
      config.input_inertia_g_mm2.size() != count ||
      config.reduction_ratio.size() != count)
    {
      throw std::invalid_argument(
              "invalid joint, gravity or armature configuration dimensions");
    }
    const std::set<std::string> retained(config.joints.begin(),
      config.joints.end());
    if (retained.size() != count) {
      throw std::invalid_argument("duplicate retained joint");
    }
    std::map<std::string, std::string> aliases;
    const auto xml = sanitized_urdf(config.urdf, aliases);
    pinocchio::Model full;
    pinocchio::urdf::buildModelFromXML(xml, full);
    full.gravity.linear() = config.gravity;
    for (const auto & name : retained) {
      if (!full.existJointName(name) || full.getJointId(name) == 0U) {
        throw std::invalid_argument("unknown retained joint: " + name);
      }
      const auto & joint = full.joints[full.getJointId(name)];
      if (joint.nq() != 1 || joint.nv() != 1) {
        throw std::invalid_argument(
                "retained joints must use scalar coordinates: " + name);
      }
    }
    for (const auto & entry : config.locked_positions) {
      if (!full.existJointName(entry.first) ||
        full.getJointId(entry.first) == 0U ||
        retained.count(entry.first) != 0U || !std::isfinite(entry.second))
      {
        throw std::invalid_argument("invalid locked joint: " + entry.first);
      }
    }
    Eigen::VectorXd locked_q = pinocchio::neutral(full);
    std::vector<pinocchio::JointIndex> locked;
    for (pinocchio::JointIndex id = 1U;
      id < static_cast<pinocchio::JointIndex>(full.njoints); ++id)
    {
      const auto & joint = full.joints[id];
      if (retained.count(full.names[id]) == 0U) {
        const auto pose = config.locked_positions.find(full.names[id]);
        if (pose == config.locked_positions.end()) {
          throw std::invalid_argument(
                  "explicit lock position required: " +
                  full.names[id]);
        }
        if (joint.nq() == 1 && joint.nv() == 1) {
          locked_q[joint.idx_q()] = pose->second;
        } else if (joint.nq() == 2 && joint.nv() == 1) {
          // Pinocchio represents one-DoF continuous joints on the unit circle.
          locked_q[joint.idx_q()] = std::cos(pose->second);
          locked_q[joint.idx_q() + 1] = std::sin(pose->second);
        } else {
          throw std::invalid_argument(
                  "locked joint is not scalar or continuous: " +
                  full.names[id]);
        }
        locked.push_back(id);
      }
    }
    model_ = pinocchio::buildReducedModel(full, locked, locked_q);
    if (model_.nv != static_cast<int>(count) || model_.nq != model_.nv) {
      throw std::invalid_argument("unexpected reduced model dimensions");
    }
    for (std::size_t axis = 0U; axis < count; ++axis) {
      const auto id = model_.getJointId(config.joints[axis]);
      const auto index = model_.joints[id].idx_v();
      const double input = config.input_inertia_g_mm2[axis];
      const double ratio = config.reduction_ratio[axis];
      const double reflected = input * 1e-9 * ratio * ratio;
      if (!std::isfinite(input) || input < 0.0 || !std::isfinite(ratio) ||
        ratio <= 0.0 || !std::isfinite(reflected))
      {
        throw std::invalid_argument("invalid input inertia or reduction ratio");
      }
      indices_.push_back(index);
      model_.armature[index] = reflected;
    }
    std::string frame = config.payload_frame;
    if (aliases.count(frame) != 0U) {
      frame = aliases.at(frame);
    }
    if (!model_.existFrame(frame)) {
      throw std::invalid_argument("unknown payload frame: " + frame);
    }
    const auto fid = model_.getFrameId(frame);
    payload_joint_ = model_.frames[fid].parentJoint;
    payload_placement_ = model_.frames[fid].placement;
    base_inertia_ = model_.inertias[payload_joint_];
    q_ = Eigen::VectorXd::Zero(model_.nq);
    zero_ = Eigen::VectorXd::Zero(model_.nv);
    joints_ = config.joints;
    data_ = std::make_unique<pinocchio::Data>(model_);
    return Result::kOk;
  } catch (const std::exception & error) {
    reason = error.what();
    data_.reset();
    indices_.clear();
    joints_.clear();
    return Result::kInvalidModel;
  }
}
} // namespace rt_arm_dynamics
