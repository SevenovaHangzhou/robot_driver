#include "rolling_trajectory_controller/envelope_loader.hpp"

#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "rolling_trajectory_controller/rolling_types.hpp"

namespace rolling_trajectory_controller
{
namespace
{

const std::array<std::string, 2> kRootKeys = {"metadata", "axes"};
const std::array<std::string, 7> kMetadataKeys = {
  "schema_version", "limits_source", "status", "estimation_method", "owner",
  "commissioning_schedule", "warning"};
const std::array<std::string, 11> kAxisKeys = {
  "name", "position_lower", "position_upper", "velocity_positive",
  "velocity_negative", "acceleration_positive", "acceleration_negative",
  "stop_acceleration_positive", "stop_acceleration_negative",
  "position_margin_lower", "position_margin_upper"};

template<std::size_t Size>
bool hasExactKeys(
  const YAML::Node & node, const std::array<std::string, Size> & expected,
  const std::string & context, std::string & error)
{
  if (!node.IsMap() || node.size() != expected.size()) {
    error = context + " must contain exactly " + std::to_string(expected.size()) + " keys";
    return false;
  }
  for (const auto & entry : node) {
    if (!entry.first.IsScalar()) {
      error = context + " contains a non-scalar key";
      return false;
    }
    const std::string key = entry.first.as<std::string>();
    if (std::find(expected.begin(), expected.end(), key) == expected.end()) {
      error = context + " contains unknown key '" + key + "'";
      return false;
    }
  }
  for (const std::string & key : expected) {
    if (!node[key]) {
      error = context + " is missing key '" + key + "'";
      return false;
    }
  }
  return true;
}

bool readFinite(
  const YAML::Node & axis, const char * field, double & value,
  const std::string & context, std::string & error)
{
  value = axis[field].as<double>();
  if (!std::isfinite(value)) {
    error = context + "." + field + " must be finite";
    return false;
  }
  return true;
}

bool readPositive(
  const YAML::Node & axis, const char * field, double & value,
  const std::string & context, std::string & error)
{
  if (!readFinite(axis, field, value, context, error)) {
    return false;
  }
  if (value <= 0.0) {
    error = context + "." + field + " must be positive";
    return false;
  }
  return true;
}

bool parseAxis(
  const YAML::Node & node, std::size_t index, AxisEnvelope & axis,
  std::string & error)
{
  const std::string context = "axes[" + std::to_string(index) + "]";
  if (!hasExactKeys(node, kAxisKeys, context, error)) {
    return false;
  }
  const std::string name = node["name"].as<std::string>();
  if (name != kJointNames[index]) {
    error = context + ".name must be '" + kJointNames[index] + "', got '" + name + "'";
    return false;
  }
  return
    readFinite(node, "position_lower", axis.position_lower, context, error) &&
    readFinite(node, "position_upper", axis.position_upper, context, error) &&
    readPositive(node, "velocity_positive", axis.velocity_positive, context, error) &&
    readPositive(node, "velocity_negative", axis.velocity_negative, context, error) &&
    readPositive(
      node, "acceleration_positive", axis.acceleration_positive, context, error) &&
    readPositive(
      node, "acceleration_negative", axis.acceleration_negative, context, error) &&
    readPositive(
      node, "stop_acceleration_positive", axis.stop_acceleration_positive,
      context, error) &&
    readPositive(
      node, "stop_acceleration_negative", axis.stop_acceleration_negative,
      context, error) &&
    readPositive(
      node, "position_margin_lower", axis.position_margin_lower, context, error) &&
    readPositive(
      node, "position_margin_upper", axis.position_margin_upper, context, error);
}

bool validateMetadata(const YAML::Node & metadata, std::string & error)
{
  if (!hasExactKeys(metadata, kMetadataKeys, "metadata", error)) {
    return false;
  }
  if (metadata["schema_version"].as<std::uint32_t>() != 1U) {
    error = "metadata.schema_version must be 1";
    return false;
  }
  if (metadata["limits_source"].as<std::string>() != "provisional") {
    error = "metadata.limits_source must be provisional";
    return false;
  }
  if (metadata["status"].as<std::string>() != "ESTIMATED_NOT_MEASURED") {
    error = "metadata.status must be ESTIMATED_NOT_MEASURED";
    return false;
  }
  for (const char * field : {
      "estimation_method", "owner", "commissioning_schedule", "warning"})
  {
    if (metadata[field].as<std::string>().empty()) {
      error = std::string("metadata.") + field + " must not be empty";
      return false;
    }
  }
  const std::string warning = metadata["warning"].as<std::string>();
  if (warning.find("DOES NOT REPLACE BENCH MEASUREMENT") == std::string::npos) {
    error = "metadata.warning must explicitly state that the file does not replace bench measurement";
    return false;
  }
  return true;
}

}  // namespace

bool loadProvisionalEnvelope(
  const std::string & path, DynamicEnvelope & envelope,
  std::string & error) noexcept
{
  envelope = DynamicEnvelope{};
  error.clear();
  try {
    if (path.empty()) {
      error = "provisional envelope path is empty";
      return false;
    }
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
      error = "cannot open provisional envelope '" + path + "'";
      return false;
    }
    const std::string bytes{
      std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
    if (bytes.empty() || stream.bad()) {
      error = "cannot read complete provisional envelope '" + path + "'";
      return false;
    }

    DynamicEnvelope candidate;
    candidate.source = LimitsSource::kProvisional;
    if (SHA256(
        reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size(),
        candidate.limits_version.data()) == nullptr)
    {
      error = "SHA-256 failed for provisional envelope '" + path + "'";
      return false;
    }

    const YAML::Node root = YAML::Load(bytes);
    if (!hasExactKeys(root, kRootKeys, "root", error) ||
      !validateMetadata(root["metadata"], error))
    {
      return false;
    }
    const YAML::Node axes = root["axes"];
    if (!axes.IsSequence() || axes.size() != kAxisCount) {
      error = "axes must be a sequence of exactly 14 entries";
      return false;
    }
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      if (!parseAxis(axes[axis], axis, candidate.axes[axis], error)) {
        return false;
      }
    }

    LimitChecker checker;
    if (!checker.configure(candidate, false, true)) {
      error = "provisional envelope fails dynamic-limit invariants";
      return false;
    }
    envelope = candidate;
    return true;
  } catch (const YAML::Exception & exception) {
    error = std::string("invalid provisional envelope YAML: ") + exception.what();
  } catch (const std::exception & exception) {
    error = std::string("cannot load provisional envelope: ") + exception.what();
  } catch (...) {
    error = "cannot load provisional envelope: unknown error";
  }
  envelope = DynamicEnvelope{};
  return false;
}

}  // namespace rolling_trajectory_controller
