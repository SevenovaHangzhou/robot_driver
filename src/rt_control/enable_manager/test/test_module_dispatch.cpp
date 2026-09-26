// Copyright 2026 kkozia
// Licensed under the Apache License, Version 2.0
//
// BQ-154 / ELECTRI-150: pure functional-module planning rules.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "enable_manager/module_dispatch.hpp"

namespace enable_manager
{
namespace
{

const std::vector<std::string> kArms{"arms"};
const std::vector<std::string> kArmsUpdown{"arms", "updown"};
const std::string kHead{"head_gimbal"};

TEST(ModuleDispatch, EmptyRequestKeepsWholeManagerAndIncludesRunningRemote)
{
  const auto plan = planModules(kArms, kHead, {});
  EXPECT_TRUE(plan.error.empty());
  EXPECT_TRUE(plan.run_local);
  EXPECT_EQ(plan.remote, RemoteSelection::kIfAvailable);
}

TEST(ModuleDispatch, EmptyRequestWithoutRemoteIsLegacyWholeManager)
{
  const auto plan = planModules({}, "", {});
  EXPECT_TRUE(plan.error.empty());
  EXPECT_TRUE(plan.run_local);
  EXPECT_EQ(plan.remote, RemoteSelection::kNone);
}

TEST(ModuleDispatch, AllOwnedModulesRunLocallyOnly)
{
  const auto plan = planModules(kArmsUpdown, kHead, {"updown", "arms", "arms"});
  EXPECT_TRUE(plan.error.empty());
  EXPECT_TRUE(plan.run_local);
  EXPECT_EQ(plan.remote, RemoteSelection::kNone);
}

TEST(ModuleDispatch, RemoteOnlyLeavesLocalAxesUntouched)
{
  const auto plan = planModules(kArms, kHead, {"head_gimbal"});
  EXPECT_TRUE(plan.error.empty());
  EXPECT_FALSE(plan.run_local);
  EXPECT_EQ(plan.remote, RemoteSelection::kRequired);
}

TEST(ModuleDispatch, LocalAndRemoteTogether)
{
  const auto plan = planModules(kArms, kHead, {"arms", "head_gimbal"});
  EXPECT_TRUE(plan.error.empty());
  EXPECT_TRUE(plan.run_local);
  EXPECT_EQ(plan.remote, RemoteSelection::kRequired);
}

TEST(ModuleDispatch, PartialOwnedSelectionFailsClosed)
{
  const auto plan = planModules(kArmsUpdown, kHead, {"arms", "head_gimbal"});
  EXPECT_EQ(plan.error, kStageModulePartitionUnsupported);
  EXPECT_FALSE(plan.run_local);
  EXPECT_EQ(plan.remote, RemoteSelection::kNone);
}

TEST(ModuleDispatch, UnknownOrUnloadedModulesAreRejected)
{
  EXPECT_EQ(planModules(kArms, kHead, {"swerve_chassis"}).error, kStageModuleNotManaged);
  EXPECT_EQ(planModules(kArms, kHead, {""}).error, kStageModuleNotManaged);
  // Without a configured remote the head name is unknown here.
  EXPECT_EQ(planModules(kArms, "", {"head_gimbal"}).error, kStageModuleNotManaged);
  // Legacy manager without module names cannot be addressed by name.
  EXPECT_EQ(planModules({}, "", {"arms"}).error, kStageModuleNotManaged);
}

TEST(ModuleDispatch, ConfigurationValidation)
{
  EXPECT_TRUE(validateModuleConfiguration(kArmsUpdown, kHead).empty());
  EXPECT_TRUE(validateModuleConfiguration({}, "").empty());
  EXPECT_FALSE(validateModuleConfiguration({"arms", "arms"}, kHead).empty());
  EXPECT_FALSE(validateModuleConfiguration({"arms", ""}, kHead).empty());
  EXPECT_FALSE(validateModuleConfiguration({"arms", "head_gimbal"}, kHead).empty());
}

TEST(ModuleDispatch, LocalNamesFallBackForLegacyManagers)
{
  EXPECT_EQ(localModuleNames({}), std::vector<std::string>{kLegacyLocalModuleName});
  EXPECT_EQ(localModuleNames(kArmsUpdown), kArmsUpdown);
}

TEST(ModuleDispatch, AggregateReportsFirstFailure)
{
  EXPECT_FALSE(aggregateOutcomes({}).ok);
  const auto ok = aggregateOutcomes({{"arms", true, "success"}, {"head_gimbal", true, "x"}});
  EXPECT_TRUE(ok.ok);
  EXPECT_EQ(ok.stage, "success");
  const auto failed = aggregateOutcomes(
    {{"arms", true, "success"}, {"head_gimbal", false, kStageModuleUnavailable}});
  EXPECT_FALSE(failed.ok);
  EXPECT_EQ(failed.stage, kStageModuleUnavailable);
}

}  // namespace
}  // namespace enable_manager
