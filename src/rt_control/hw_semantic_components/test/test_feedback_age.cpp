#include <gtest/gtest.h>

#include <limits>

#include "hw_semantic_components/feedback_age.hpp"

namespace hw_semantic_components
{
namespace
{

TEST(FeedbackAgeTest, FreshOnlyForFiniteNonNegativeAgesWithinTheLimit)
{
  EXPECT_TRUE(feedback_fresh(0.0, 100.0));
  EXPECT_TRUE(feedback_fresh(100.0, 100.0));
  EXPECT_FALSE(feedback_fresh(100.5, 100.0));
  EXPECT_FALSE(feedback_fresh(-1.0, 100.0));
  EXPECT_FALSE(feedback_fresh(std::numeric_limits<double>::infinity(), 100.0));
  EXPECT_FALSE(feedback_fresh(std::numeric_limits<double>::quiet_NaN(), 100.0));
}

}  // namespace
}  // namespace hw_semantic_components
