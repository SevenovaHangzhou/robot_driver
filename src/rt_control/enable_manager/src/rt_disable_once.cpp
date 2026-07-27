#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/rt_enable.hpp"

namespace
{
using SteadyClock = std::chrono::steady_clock;

constexpr auto kTotalDeadline = std::chrono::seconds(30);
constexpr auto kServiceProbe = std::chrono::milliseconds(100);
constexpr auto kControllerSwitchTimeout = std::chrono::seconds(4);
constexpr char kControllerManagerPrefix[] = "/controller_manager";
constexpr char kEthercatHardwareName[] = "ecat_arms";

template<typename ServiceT>
bool waitForService(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const SteadyClock::time_point deadline)
{
  while (rclcpp::ok() && SteadyClock::now() < deadline) {
    const auto remaining = deadline - SteadyClock::now();
    const auto probe = std::chrono::duration_cast<SteadyClock::duration>(kServiceProbe);
    if (client->wait_for_service(std::min(probe, remaining))) {
      return true;
    }
  }
  return false;
}

template<typename FutureT>
bool waitForResponse(
  const rclcpp::Node::SharedPtr & node, FutureT & future,
  const SteadyClock::time_point deadline)
{
  const auto now = SteadyClock::now();
  if (!rclcpp::ok() || now >= deadline) {
    return false;
  }
  return rclcpp::spin_until_future_complete(node, future, deadline - now) ==
         rclcpp::FutureReturnCode::SUCCESS;
}

bool disableEthercatAxes(
  const rclcpp::Node::SharedPtr & node, const SteadyClock::time_point deadline)
{
  const auto client = node->create_client<robot_interfaces::srv::RtEnable>("/rt/disable");
  if (!waitForService<robot_interfaces::srv::RtEnable>(client, deadline)) {
    std::cerr << "UNCLEAN_SHUTDOWN: /rt/disable unavailable within shutdown deadline\n";
    return false;
  }

  const auto request = std::make_shared<robot_interfaces::srv::RtEnable::Request>();
  auto future = client->async_send_request(request);
  if (!waitForResponse(node, future, deadline)) {
    std::cerr << "UNCLEAN_SHUTDOWN: /rt/disable did not return within shutdown deadline\n";
    return false;
  }

  const auto response = future.get();
  std::cerr << "rt_control shutdown disable result: ok=" << std::boolalpha << response->ok
            << " stage=" << response->stage
            << " failed_batch=" << static_cast<int>(response->failed_batch)
            << " failed_joint=" << response->failed_joint
            << " status_word=" << response->status_word << '\n';
  return response->ok;
}

bool quiesceEthercatControllers(
  const rclcpp::Node::SharedPtr & node, const SteadyClock::time_point deadline)
{
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  const auto client = node->create_client<SwitchController>(
    std::string(kControllerManagerPrefix) + "/switch_controller");
  if (!waitForService<SwitchController>(client, deadline)) {
    std::cerr <<
      "UNCLEAN_SHUTDOWN: controller-manager switch service unavailable within shutdown deadline\n";
    return false;
  }

  const auto request = std::make_shared<SwitchController::Request>();
  request->deactivate_controllers = {"enable_manager", "joint_state_broadcaster"};
  request->strictness = SwitchController::Request::STRICT;
  request->activate_asap = false;
  request->timeout.sec = static_cast<std::int32_t>(kControllerSwitchTimeout.count());
  request->timeout.nanosec = 0U;
  auto future = client->async_send_request(request);
  if (!waitForResponse(node, future, deadline)) {
    std::cerr <<
      "UNCLEAN_SHUTDOWN: EtherCAT controller quiesce did not return within shutdown deadline\n";
    return false;
  }
  if (!future.get()->ok) {
    std::cerr << "UNCLEAN_SHUTDOWN: EtherCAT controller quiesce was rejected\n";
    return false;
  }

  std::cerr <<
    "rt_control shutdown controllers quiesced: enable_manager,joint_state_broadcaster\n";
  return true;
}

bool deactivateEthercatHardware(
  const rclcpp::Node::SharedPtr & node, const SteadyClock::time_point deadline)
{
  using SetHardwareState = controller_manager_msgs::srv::SetHardwareComponentState;
  const auto client = node->create_client<SetHardwareState>(
    std::string(kControllerManagerPrefix) + "/set_hardware_component_state");
  if (!waitForService<SetHardwareState>(client, deadline)) {
    std::cerr <<
      "UNCLEAN_SHUTDOWN: hardware-state service unavailable within shutdown deadline\n";
    return false;
  }

  const auto request = std::make_shared<SetHardwareState::Request>();
  request->name = kEthercatHardwareName;
  request->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  request->target_state.label = "inactive";
  auto future = client->async_send_request(request);
  if (!waitForResponse(node, future, deadline)) {
    std::cerr <<
      "UNCLEAN_SHUTDOWN: EtherCAT hardware deactivate did not return within shutdown deadline\n";
    return false;
  }

  const auto response = future.get();
  const bool inactive =
    response->ok && response->state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  if (!inactive) {
    std::cerr << "UNCLEAN_SHUTDOWN: ecat_arms did not reach inactive; ok=" << std::boolalpha
              << response->ok << " state_id=" << static_cast<unsigned int>(response->state.id)
              << " state_label=" << response->state.label << '\n';
    return false;
  }

  std::cerr << "rt_control shutdown EtherCAT hardware state: inactive\n";
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared("rt_shutdown_once");
  const auto deadline = SteadyClock::now() + kTotalDeadline;

  // Keep progressing toward PREOP even when the group disable reports a pre-existing
  // drive Fault. The nonzero exit still prevents that case from being called clean.
  const bool disable_clean = disableEthercatAxes(node, deadline);
  const bool controllers_quiesced = quiesceEthercatControllers(node, deadline);
  const bool ethercat_inactive =
    controllers_quiesced && deactivateEthercatHardware(node, deadline);

  const bool clean = disable_clean && controllers_quiesced && ethercat_inactive;
  rclcpp::shutdown();
  return clean ? EXIT_SUCCESS : EXIT_FAILURE;
}
