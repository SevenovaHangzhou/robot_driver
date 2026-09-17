#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rt_control_semantic_components/force_torque_sensor.hpp"

namespace rt_control_semantic_components
{
namespace
{

const std::array<std::string, 6> kValueNames{
  "sensor/channel_1_raw", "sensor/channel_2_raw", "sensor/channel_3_raw",
  "sensor/channel_4_raw", "sensor/channel_5_raw", "sensor/channel_6_raw"};
const std::vector<std::string> kAuxiliaryNames{
  "sensor/sample_code_1_raw", "sensor/sample_code_2_raw",
  "sensor/sample_code_3_raw", "sensor/sample_code_4_raw",
  "sensor/sample_code_5_raw", "sensor/sample_code_6_raw"};

struct BoundSensor
{
  std::array<double, 6> values{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  std::array<double, 6> auxiliary{7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
  double unrelated_value{-1.0};
  std::vector<hardware_interface::StateInterface> handles;
  std::vector<hardware_interface::LoanedStateInterface> loaned;

  BoundSensor()
  {
    handles.emplace_back("other", "state", &unrelated_value);
    for (std::size_t index = 0; index < values.size(); ++index) {
      const auto slash = kValueNames[index].find_last_of('/');
      handles.emplace_back(
        kValueNames[index].substr(0, slash),
        kValueNames[index].substr(slash + 1), &values[index]);
    }
    for (std::size_t index = 0; index < auxiliary.size(); ++index) {
      const auto slash = kAuxiliaryNames[index].find_last_of('/');
      handles.emplace_back(
        kAuxiliaryNames[index].substr(0, slash),
        kAuxiliaryNames[index].substr(slash + 1), &auxiliary[index]);
    }
    for (auto iterator = handles.rbegin(); iterator != handles.rend(); ++iterator) {
      loaned.emplace_back(*iterator);
    }
  }
};

TEST(ForceTorqueSensorTest, DeclaresOrderedValueAndAuxiliaryInterfaces)
{
  const ForceTorqueSensor sensor{kValueNames, kAuxiliaryNames};
  const auto & names = sensor.get_state_interface_names();

  ASSERT_EQ(names.size(), 12U);
  EXPECT_TRUE(std::equal(kValueNames.begin(), kValueNames.end(), names.begin()));
  EXPECT_TRUE(
    std::equal(
      kAuxiliaryNames.begin(), kAuxiliaryNames.end(), names.begin() + 6));
}

TEST(ForceTorqueSensorTest, BindsByFullNameAndReadsOneAllocationFreeSnapshot)
{
  BoundSensor resources;
  ForceTorqueSensor sensor{kValueNames, kAuxiliaryNames};

  ASSERT_TRUE(sensor.assign_loaned_state_interfaces(resources.loaned));
  ASSERT_TRUE(sensor.is_bound());
  const auto sample = sensor.read();
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->values, resources.values);
  EXPECT_EQ(sample->auxiliary_count, resources.auxiliary.size());
  EXPECT_EQ(sample->auxiliary, resources.auxiliary);
}

TEST(ForceTorqueSensorTest, RejectsMissingAndDuplicateInterfacesWithoutPartialBinding)
{
  BoundSensor resources;
  ForceTorqueSensor missing{kValueNames, kAuxiliaryNames};
  resources.loaned.clear();
  for (auto iterator = resources.handles.rbegin(); iterator != resources.handles.rend();
    ++iterator)
  {
    if (iterator->get_name() != kAuxiliaryNames.back()) {
      resources.loaned.emplace_back(*iterator);
    }
  }
  EXPECT_FALSE(missing.assign_loaned_state_interfaces(resources.loaned));
  EXPECT_FALSE(missing.is_bound());
  EXPECT_FALSE(missing.read().has_value());

  BoundSensor duplicates;
  duplicates.loaned.emplace_back(duplicates.handles[1]);
  ForceTorqueSensor ambiguous{kValueNames, kAuxiliaryNames};
  EXPECT_FALSE(ambiguous.assign_loaned_state_interfaces(duplicates.loaned));
  EXPECT_FALSE(ambiguous.is_bound());
}

TEST(ForceTorqueSensorTest, ReleaseIsIdempotentAndAllowsRebinding)
{
  BoundSensor resources;
  ForceTorqueSensor sensor{kValueNames, kAuxiliaryNames};
  ASSERT_TRUE(sensor.assign_loaned_state_interfaces(resources.loaned));

  sensor.release_interfaces();
  sensor.release_interfaces();
  EXPECT_FALSE(sensor.is_bound());
  EXPECT_FALSE(sensor.read().has_value());
  EXPECT_TRUE(sensor.assign_loaned_state_interfaces(resources.loaned));
  EXPECT_TRUE(sensor.is_bound());
}

TEST(ForceTorqueSensorTest, RejectsInvalidInterfaceDefinitions)
{
  auto duplicate_values = kValueNames;
  duplicate_values[5] = duplicate_values[0];
  EXPECT_THROW(
    ForceTorqueSensor(duplicate_values, kAuxiliaryNames), std::invalid_argument);

  auto too_many_auxiliary = kAuxiliaryNames;
  too_many_auxiliary.push_back("sensor/extra");
  EXPECT_THROW(
    ForceTorqueSensor(kValueNames, too_many_auxiliary), std::invalid_argument);

  auto empty_values = kValueNames;
  empty_values[0].clear();
  EXPECT_THROW(
    ForceTorqueSensor(empty_values, kAuxiliaryNames), std::invalid_argument);
}

TEST(ForceTorqueSensorTest, PreservesNonfiniteValuesForPolicyLayerRejection)
{
  BoundSensor resources;
  resources.values[2] = std::numeric_limits<double>::quiet_NaN();
  ForceTorqueSensor sensor{kValueNames, kAuxiliaryNames};
  ASSERT_TRUE(sensor.assign_loaned_state_interfaces(resources.loaned));

  const auto sample = sensor.read();
  ASSERT_TRUE(sample.has_value());
  EXPECT_TRUE(std::isnan(sample->values[2]));
}

}  // namespace
}  // namespace rt_control_semantic_components
