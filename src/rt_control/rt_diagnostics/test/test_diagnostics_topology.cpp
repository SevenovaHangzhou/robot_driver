#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "rt_diagnostics/diagnostics_topology.hpp"

namespace rt_diagnostics
{
namespace
{

TEST(DiagnosticsTopologyTest, PreservesConfiguredOrderForNoncontiguousTopology)
{
  const auto topology = DiagnosticsTopology::create(
    {"axis_c", "axis_a", "axis_b"}, {7, 1, 11}, 13, {42, 7, 11});

  ASSERT_EQ(topology.ethercat_axes().size(), 3U);
  EXPECT_EQ(topology.ethercat_axes()[0].joint_name, "axis_c");
  EXPECT_EQ(topology.ethercat_axes()[0].ring_position, 7U);
  EXPECT_EQ(topology.ethercat_axes()[2].joint_name, "axis_b");
  EXPECT_EQ(topology.ethercat_axes()[2].ring_position, 11U);
  EXPECT_EQ(topology.ethercat_expected_responders(), 13U);
  EXPECT_EQ(topology.canopen_node_ids(), (std::vector<std::uint8_t>{42U, 7U, 11U}));
}

TEST(DiagnosticsTopologyTest, RejectsHardwareIdsThatCanFeedBackAsConfiguredCanNodes)
{
  const auto topology = DiagnosticsTopology::create(
    {"axis_a"}, {4}, 5, {2, 11});

  EXPECT_NO_THROW(topology.validate_hardware_id("robot-001"));
  EXPECT_THROW(topology.validate_hardware_id(""), std::invalid_argument);
  EXPECT_THROW(topology.validate_hardware_id("2"), std::invalid_argument);
  EXPECT_THROW(topology.validate_hardware_id("11"), std::invalid_argument);
}

struct InvalidTopology
{
  std::vector<std::string> joint_names;
  std::vector<std::int64_t> ring_positions;
  std::int64_t expected_responders;
  std::vector<std::int64_t> node_ids;
};

class InvalidDiagnosticsTopologyTest : public ::testing::TestWithParam<InvalidTopology>
{};

TEST_P(InvalidDiagnosticsTopologyTest, RejectsUnsafeOrAmbiguousTopology)
{
  const auto & topology = GetParam();
  EXPECT_THROW(
    DiagnosticsTopology::create(
      topology.joint_names, topology.ring_positions,
      topology.expected_responders, topology.node_ids),
    std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(
  InvalidTopologies, InvalidDiagnosticsTopologyTest,
  ::testing::Values(
    InvalidTopology{{}, {}, 1, {2}},
    InvalidTopology{{"a"}, {}, 1, {2}},
    InvalidTopology{{"a", "a"}, {0, 1}, 2, {2}},
    InvalidTopology{{"a", "b"}, {0, 0}, 2, {2}},
    InvalidTopology{{"a"}, {-1}, 1, {2}},
    InvalidTopology{{"a"}, {1}, 1, {2}},
    InvalidTopology{{"a", "b"}, {0, 1}, 1, {2}},
    InvalidTopology{{"a"}, {0}, 0, {2}},
    InvalidTopology{{"a"}, {0}, 1, {}},
    InvalidTopology{{"a"}, {0}, 1, {0}},
    InvalidTopology{{"a"}, {0}, 1, {128}},
    InvalidTopology{{"a"}, {0}, 1, {2, 2}}));

}  // namespace
}  // namespace rt_diagnostics
