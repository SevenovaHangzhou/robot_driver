#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"

#include <array>
#include <iomanip>
#include <sstream>
#include <string>

namespace ctrl_rolling_trajectory
{

void RollingTrajectoryController::publishRejection(const BatchMessage & message)
{
  // Called only by the serialized non-RT batch callback, never by update().
  if (!rejection_publisher_ || !rejection_publisher_->is_activated()) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray report;
  report.header.stamp = get_node()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.name = rejection_publisher_->get_topic_name();
  status.hardware_id = get_node()->get_name();
  status.message = "Rolling batch rejected";
  const auto add = [&status](const std::string & key, const auto & value) {
    std::ostringstream text;
    text << std::setprecision(17) << value;
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = text.str();
    status.values.push_back(item);
  };
  add("epoch", message.epoch);
  add("sequence", message.sequence);
  add("result", static_cast<unsigned int>(last_result_));
  if (last_result_ == StateMessage::NOT_ACCEPTING) {
    add("reason", admission_failure_);
  }
  const auto & detail = batch_rejection_;
  if (detail.quantity != LimitQuantity::kNone && detail.axis < kAxisCount) {
    constexpr std::array<const char *, 9> quantities = {
      "unknown",           "position_lower",        "position_upper",        "velocity_positive",
      "velocity_negative", "acceleration_positive", "acceleration_negative", "stopping_lower",
      "stopping_upper"};
    add("joint", runtime_configuration_.joints[detail.axis]);
    add("axis", detail.axis);
    add("segment", detail.segment);
    add("start_ns", detail.start_ns);
    add("end_ns", detail.end_ns);
    add("quantity", quantities[static_cast<std::size_t>(detail.quantity)]);
    add("value", detail.value);
    add("limit", detail.limit);
  }
  report.status.push_back(status);
  rejection_publisher_->publish(report);
}

}  // namespace ctrl_rolling_trajectory
