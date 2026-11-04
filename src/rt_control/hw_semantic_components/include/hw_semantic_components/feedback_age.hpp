#ifndef HW_SEMANTIC_COMPONENTS__FEEDBACK_AGE_HPP_
#define HW_SEMANTIC_COMPONENTS__FEEDBACK_AGE_HPP_

#include <cmath>

namespace hw_semantic_components
{

// Feedback-age state interfaces (`<joint>/feedback_age_ms`, `ethercat_domain/process_data_age_ms`)
// report milliseconds since the last valid sample; a non-finite or negative value means none.
inline bool feedback_fresh(double age_ms, double limit_ms) noexcept
{
  return std::isfinite(age_ms) && age_ms >= 0.0 && age_ms <= limit_ms;
}

}  // namespace hw_semantic_components

#endif  // HW_SEMANTIC_COMPONENTS__FEEDBACK_AGE_HPP_
