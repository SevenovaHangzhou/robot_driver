#include <gtest/gtest.h>

#include <limits>

#include "hw_ethercat/pp_axis.hpp"

using hw_ethercat::PpAxis;
using hw_ethercat::PpAxisConfig;
using hw_ethercat::PpAxisRequest;
using hw_ethercat::PpFeedback;
using hw_ethercat::PpState;

namespace
{
// Synthetic calibration and timing, never a production drive configuration.
PpAxisConfig config()
{
  return {1000.0, 50.0, 0.0, 0.18, 0.001, 0.001, 0.1, 0.2, 2};
}

class PpAxisTest : public testing::Test
{
protected:
  PpAxis axis{config()};
  PpAxisRequest request{0, 0.0, true};
  // Enabled control word, operation-enabled status, PP mode, at raw 50 (position 0).
  PpFeedback feedback{true, 0x000f, 0x0027, 1, 50, 0.0};
  double time{0.0};

  auto tick(bool sent = true, bool extra_valid = true)
  {
    if (sent) {
      axis.sent();
    }
    time += 0.001;
    return axis.update(request, feedback, time, extra_valid);
  }
};
}  // namespace

TEST(PpAxisConfiguration, RejectsInvalidCalibration)
{
  auto cfg = config();
  cfg.counts_per_unit = 0.0;
  EXPECT_THROW(PpAxis{cfg}, std::invalid_argument);
  cfg = config();
  cfg.max_position = cfg.min_position;
  EXPECT_THROW(PpAxis{cfg}, std::invalid_argument);
  cfg = config();
  cfg.ack_timeout = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PpAxis{cfg}, std::invalid_argument);
}

TEST_F(PpAxisTest, IdleHoldsWithHaltAndTracksTheMeasuredPosition)
{
  feedback.actual_position = 123;
  const auto output = tick();
  EXPECT_EQ(output.state, PpState::idle);
  EXPECT_EQ(output.target, 123);
  EXPECT_EQ(output.control_word, 0x010f);
  EXPECT_FALSE(axis.accepted());
}

TEST_F(PpAxisTest, NewTargetRunsTheSetpointHandshakeToReached)
{
  tick();
  request = {1, 0.1, false};
  const auto prepared = tick();
  EXPECT_TRUE(axis.accepted());
  EXPECT_EQ(prepared.target, 150);                    // zero 50 + 0.1 * 1000
  EXPECT_EQ(prepared.control_word & 0x0110, 0x0100);  // target first, edge next cycle
  EXPECT_EQ(tick().control_word & 0x0110, 0x0010);    // new set-point edge, halt released
  EXPECT_FALSE(axis.accepted());
  feedback.status_word |= 0x1000;  // set-point acknowledge
  EXPECT_EQ(tick().control_word & 0x0010, 0);
  feedback.status_word &= ~0x1000;
  EXPECT_EQ(tick().state, PpState::moving);
  feedback.actual_position = 150;
  feedback.status_word |= 0x0400;  // target reached
  const auto reached = tick();
  EXPECT_EQ(reached.state, PpState::reached);
  EXPECT_EQ(reached.sequence, 1u);
}

TEST_F(PpAxisTest, AdapterRejectionAndOutOfRangeTargetsFault)
{
  tick();
  request = {1, 0.1, false};
  EXPECT_EQ(tick(true, false).state, PpState::fault);

  PpAxis other{config()};
  axis.sent();
  other.update({0, 0.0, true}, feedback, 0.001);
  other.sent();
  EXPECT_EQ(other.update({1, 0.2, false}, feedback, 0.002).state, PpState::fault);
}

TEST_F(PpAxisTest, MissingAcknowledgeTimesOut)
{
  tick();
  request = {1, 0.1, false};
  tick();
  tick();
  time += 0.2;
  EXPECT_EQ(tick().state, PpState::fault);
}

TEST_F(PpAxisTest, DisabledDriveNeverReceivesAnEnablingWord)
{
  feedback.control_word = 0x0006;  // shutdown
  const auto output = tick();
  EXPECT_EQ(output.state, PpState::unavailable);
  EXPECT_EQ(output.control_word, 0x0006);
}
