#ifndef ENABLE_MANAGER__MODULE_DISPATCH_HPP_
#define ENABLE_MANAGER__MODULE_DISPATCH_HPP_

// BQ-154 / ELECTRI-150 step 1: functional-module selection for /rt/enable,
// /rt/disable and /rt/reset_fault. Pure logic without ROS so it can be tested
// in isolation.
//
// The local enable_manager state machine still moves all of its axes together.
// A request that names only part of the manager's own modules is therefore
// rejected (fail closed) instead of being widened or narrowed silently. The
// optional remote module (head_gimbal) is served by its own manager and is
// forwarded to its /rt/head/* services.

#include <algorithm>
#include <string>
#include <vector>

namespace enable_manager
{

inline constexpr const char * kStageModuleNotManaged = "module_not_managed";
inline constexpr const char * kStageModulePartitionUnsupported = "module_partition_unsupported";
inline constexpr const char * kStageModuleUnavailable = "module_unavailable";
inline constexpr const char * kStageModuleServiceTimeout = "module_service_timeout";
inline constexpr const char * kLegacyLocalModuleName = "managed_axes";

enum class RemoteSelection
{
  kNone,         // do not touch the remote module
  kIfAvailable,  // whole-robot request: include the remote module when it is running
  kRequired      // explicitly requested: a missing service is a failure
};

struct ModulePlan
{
  std::string error;  // empty when the request is valid
  bool run_local{false};
  RemoteSelection remote{RemoteSelection::kNone};
};

struct ModuleOutcome
{
  std::string name;
  bool ok{false};
  std::string stage;
};

struct AggregateOutcome
{
  bool ok{false};
  std::string stage;
};

// Returns an error string when the configuration itself is inconsistent.
inline std::string validateModuleConfiguration(
  const std::vector<std::string> & owned, const std::string & remote_module)
{
  std::vector<std::string> sorted = owned;
  std::sort(sorted.begin(), sorted.end());
  if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) {
    return "owned_modules must be unique";
  }
  for (const auto & name : owned) {
    if (name.empty()) {
      return "owned_modules must not contain empty names";
    }
    if (!remote_module.empty() && name == remote_module) {
      return "remote_module_name must not also be an owned module";
    }
  }
  return {};
}

inline ModulePlan planModules(
  const std::vector<std::string> & owned, const std::string & remote_module,
  const std::vector<std::string> & requested)
{
  ModulePlan plan;
  if (requested.empty()) {
    plan.run_local = true;
    plan.remote = remote_module.empty() ? RemoteSelection::kNone : RemoteSelection::kIfAvailable;
    return plan;
  }
  std::vector<std::string> local_selected;
  for (const auto & name : requested) {
    if (!remote_module.empty() && name == remote_module) {
      plan.remote = RemoteSelection::kRequired;
    } else if (std::find(owned.begin(), owned.end(), name) != owned.end()) {
      if (std::find(local_selected.begin(), local_selected.end(), name) == local_selected.end()) {
        local_selected.push_back(name);
      }
    } else {
      plan = ModulePlan{};
      plan.error = kStageModuleNotManaged;
      return plan;
    }
  }
  if (!local_selected.empty() && local_selected.size() != owned.size()) {
    plan = ModulePlan{};
    plan.error = kStageModulePartitionUnsupported;
    return plan;
  }
  plan.run_local = !local_selected.empty();
  return plan;
}

// Names reported for the local manager's outcome.
inline std::vector<std::string> localModuleNames(const std::vector<std::string> & owned)
{
  return owned.empty() ? std::vector<std::string>{kLegacyLocalModuleName} : owned;
}

inline AggregateOutcome aggregateOutcomes(const std::vector<ModuleOutcome> & outcomes)
{
  AggregateOutcome result;
  result.ok = !outcomes.empty();
  for (const auto & outcome : outcomes) {
    if (!outcome.ok) {
      result.ok = false;
      result.stage = outcome.stage;
      return result;
    }
  }
  result.stage = outcomes.empty() ? std::string{} : outcomes.front().stage;
  return result;
}

}  // namespace enable_manager

#endif  // ENABLE_MANAGER__MODULE_DISPATCH_HPP_
