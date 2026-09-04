#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rt_control_semantic_components/cia402_axis.hpp"

namespace rt_control_semantic_components
{
namespace
{

constexpr char kJointName[] = "right_joint1";
constexpr char kCommandInterfaceName[] = "right_joint1/control_word";
constexpr char kStateInterfaceName[] = "right_joint1/status_word";

struct BoundAxis
{
  double unrelated_command_value{-1.0};
  double command_value{-1.0};
  double unrelated_state_value{-1.0};
  double status_value{0.0};

  hardware_interface::CommandInterface unrelated_command{
    "left_joint1", "control_word", &unrelated_command_value};
  hardware_interface::CommandInterface command{
    kJointName, "control_word", &command_value};
  hardware_interface::StateInterface unrelated_state{
    "left_joint1", "status_word", &unrelated_state_value};
  hardware_interface::StateInterface status{kJointName, "status_word", &status_value};

  std::vector<hardware_interface::LoanedCommandInterface> commands;
  std::vector<hardware_interface::LoanedStateInterface> states;

  BoundAxis()
  {
    // Deliberately put the requested resources after unrelated ones. A semantic component
    // binds by full interface name, never by vector position.
    commands.emplace_back(unrelated_command);
    commands.emplace_back(command);
    states.emplace_back(unrelated_state);
    states.emplace_back(status);
  }
};

TEST(Cia402AxisTest, DeclaresStableInterfaceNames)
{
  const Cia402Axis axis(kJointName);

  EXPECT_EQ(axis.get_name(), kJointName);
  EXPECT_EQ(
    axis.get_command_interface_names(),
    std::vector<std::string>({kCommandInterfaceName}));
  EXPECT_EQ(
    axis.get_state_interface_names(),
    std::vector<std::string>({kStateInterfaceName}));
}

TEST(Cia402AxisTest, AssignsCommandAndStateByNameFromUnorderedVectors)
{
  BoundAxis resources;
  Cia402Axis axis(kJointName);

  ASSERT_TRUE(axis.assign_loaned_command_interfaces(resources.commands));
  ASSERT_TRUE(axis.assign_loaned_state_interfaces(resources.states));
  EXPECT_TRUE(axis.has_command_interface());
  EXPECT_TRUE(axis.has_state_interface());
  EXPECT_TRUE(axis.is_bound());

  resources.status_value = 0x0027;
  ASSERT_TRUE(axis.get_status_word().has_value());
  EXPECT_EQ(*axis.get_status_word(), 0x0027U);
  EXPECT_EQ(axis.get_state(), Cia402State::kOperationEnabled);

  EXPECT_TRUE(axis.set_control_word(0x000FU));
  EXPECT_DOUBLE_EQ(resources.command_value, 15.0);
  EXPECT_DOUBLE_EQ(resources.unrelated_command_value, -1.0);
}

TEST(Cia402AxisTest, RejectsMissingInterfacesWithoutKeepingPartialBindings)
{
  double command_value{0.0};
  double state_value{0.0};
  hardware_interface::CommandInterface wrong_command{
    "left_joint1", "control_word", &command_value};
  hardware_interface::StateInterface wrong_state{
    "left_joint1", "status_word", &state_value};
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  std::vector<hardware_interface::LoanedStateInterface> states;
  commands.emplace_back(wrong_command);
  states.emplace_back(wrong_state);

  Cia402Axis axis(kJointName);
  EXPECT_FALSE(axis.assign_loaned_command_interfaces(commands));
  EXPECT_FALSE(axis.assign_loaned_state_interfaces(states));
  EXPECT_FALSE(axis.has_command_interface());
  EXPECT_FALSE(axis.has_state_interface());
  EXPECT_FALSE(axis.is_bound());
  EXPECT_FALSE(axis.set_control_word(0x0006U));
  EXPECT_FALSE(axis.get_status_word().has_value());
  EXPECT_EQ(axis.get_state(), Cia402State::kUnknown);
}

TEST(Cia402AxisTest, RejectsDuplicateInterfacesAsAmbiguous)
{
  double first_command_value{0.0};
  double second_command_value{0.0};
  double first_state_value{0.0};
  double second_state_value{0.0};
  hardware_interface::CommandInterface first_command{
    kJointName, "control_word", &first_command_value};
  hardware_interface::CommandInterface second_command{
    kJointName, "control_word", &second_command_value};
  hardware_interface::StateInterface first_state{
    kJointName, "status_word", &first_state_value};
  hardware_interface::StateInterface second_state{
    kJointName, "status_word", &second_state_value};
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  std::vector<hardware_interface::LoanedStateInterface> states;
  commands.emplace_back(first_command);
  commands.emplace_back(second_command);
  states.emplace_back(first_state);
  states.emplace_back(second_state);

  Cia402Axis axis(kJointName);
  EXPECT_FALSE(axis.assign_loaned_command_interfaces(commands));
  EXPECT_FALSE(axis.assign_loaned_state_interfaces(states));
  EXPECT_FALSE(axis.has_command_interface());
  EXPECT_FALSE(axis.has_state_interface());
  EXPECT_FALSE(axis.is_bound());
}

TEST(Cia402AxisTest, RejectsReassignmentUntilInterfacesAreReleased)
{
  BoundAxis first_resources;
  BoundAxis replacement_resources;
  Cia402Axis axis(kJointName);
  ASSERT_TRUE(axis.assign_loaned_command_interfaces(first_resources.commands));
  ASSERT_TRUE(axis.assign_loaned_state_interfaces(first_resources.states));

  replacement_resources.status_value = 0x0008;
  first_resources.status_value = 0x0027;
  EXPECT_FALSE(axis.assign_loaned_command_interfaces(replacement_resources.commands));
  EXPECT_FALSE(axis.assign_loaned_state_interfaces(replacement_resources.states));

  // A rejected rebind leaves the original, coherent pair intact.
  ASSERT_TRUE(axis.is_bound());
  EXPECT_EQ(axis.get_status_word(), 0x0027U);
  ASSERT_TRUE(axis.set_control_word(0x000FU));
  EXPECT_DOUBLE_EQ(first_resources.command_value, 15.0);
  EXPECT_DOUBLE_EQ(replacement_resources.command_value, -1.0);
}

TEST(Cia402AxisTest, ReleaseIsIdempotentAndAllowsReassignment)
{
  BoundAxis resources;
  Cia402Axis axis(kJointName);
  ASSERT_TRUE(axis.assign_loaned_command_interfaces(resources.commands));
  ASSERT_TRUE(axis.assign_loaned_state_interfaces(resources.states));

  resources.status_value = 0x0040;
  ASSERT_EQ(axis.get_status_word(), 0x0040U);
  axis.release_interfaces();
  axis.release_interfaces();

  EXPECT_FALSE(axis.has_command_interface());
  EXPECT_FALSE(axis.has_state_interface());
  EXPECT_FALSE(axis.is_bound());
  EXPECT_FALSE(axis.set_control_word(0x0006U));
  EXPECT_FALSE(axis.get_status_word().has_value());
  EXPECT_EQ(axis.get_state(), Cia402State::kUnknown);
  EXPECT_DOUBLE_EQ(resources.command_value, -1.0);

  EXPECT_TRUE(axis.assign_loaned_command_interfaces(resources.commands));
  EXPECT_TRUE(axis.assign_loaned_state_interfaces(resources.states));
  EXPECT_TRUE(axis.is_bound());
}

TEST(Cia402AxisTest, ConvertsStatusWordOnlyWhenDoubleIsAnExactUint16)
{
  BoundAxis resources;
  Cia402Axis axis(kJointName);
  ASSERT_TRUE(axis.assign_loaned_state_interfaces(resources.states));

  for (const auto expected : std::array<std::uint16_t, 3>{0U, 42U, 65535U}) {
    resources.status_value = static_cast<double>(expected);
    ASSERT_TRUE(axis.get_status_word().has_value()) << expected;
    EXPECT_EQ(*axis.get_status_word(), expected);
  }

  const std::array<double, 7> invalid_values{
    -1.0,
    0.5,
    65535.5,
    65536.0,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()};
  for (const double invalid : invalid_values) {
    resources.status_value = invalid;
    EXPECT_FALSE(axis.get_status_word().has_value()) << invalid;
    EXPECT_EQ(axis.get_state(), Cia402State::kUnknown) << invalid;
  }
}

TEST(Cia402AxisTest, WritesEveryUint16ControlWordExactlyAsDouble)
{
  BoundAxis resources;
  Cia402Axis axis(kJointName);
  ASSERT_TRUE(axis.assign_loaned_command_interfaces(resources.commands));

  for (const auto value : std::array<std::uint16_t, 3>{0U, 0x000FU, 65535U}) {
    ASSERT_TRUE(axis.set_control_word(value));
    EXPECT_DOUBLE_EQ(resources.command_value, static_cast<double>(value));
  }
}

TEST(Cia402AxisTest, DecodesEveryCia402PowerDriveState)
{
  using State = Cia402State;
  const std::array<std::pair<std::uint16_t, State>, 8> states{{
    {0x0000U, State::kNotReadyToSwitchOn},
    {0x0040U, State::kSwitchOnDisabled},
    {0x0021U, State::kReadyToSwitchOn},
    {0x0023U, State::kSwitchedOn},
    {0x0027U, State::kOperationEnabled},
    {0x0007U, State::kQuickStopActive},
    {0x000FU, State::kFaultReactionActive},
    {0x0008U, State::kFault},
  }};

  for (const auto & [status_word, expected] : states) {
    EXPECT_EQ(Cia402Axis::decode_state(status_word), expected) << status_word;
    // Bits outside the CiA402 state masks must not change the decoded state.
    EXPECT_EQ(Cia402Axis::decode_state(static_cast<std::uint16_t>(status_word | 0xFF90U)), expected)
      << status_word;
  }
  EXPECT_EQ(Cia402Axis::decode_state(0x0001U), State::kUnknown);
}

TEST(Cia402AxisTest, DecodesTheBoundStatusInterface)
{
  BoundAxis resources;
  Cia402Axis axis(kJointName);
  ASSERT_TRUE(axis.assign_loaned_state_interfaces(resources.states));

  resources.status_value = 0x0008;
  EXPECT_EQ(axis.get_state(), Cia402State::kFault);
  resources.status_value = 0x0001;
  EXPECT_EQ(axis.get_state(), Cia402State::kUnknown);
}

}  // namespace
}  // namespace rt_control_semantic_components
