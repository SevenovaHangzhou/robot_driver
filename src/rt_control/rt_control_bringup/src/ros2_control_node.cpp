// Copyright 2020 ROS2-Control Development Team
// Copyright 2026 Sevenova Hangzhou
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ros2_control 2.53.1 controller_manager/src/ros2_control_node.cpp (Humble), changed only where
// marked "rt_control": the update loop waits on steady_clock and skips missed ticks
// (`overruns.manage`, default true) as ros2_control Jazzy does, instead of running them back to back.
// Overruns are counted in the update thread with atomics and reported from a non-RT timer.

#include <errno.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"
#include "rt_control_bringup/control_loop_timing.hpp"

using namespace std::chrono_literals;

namespace
{
// Reference: https://man7.org/linux/man-pages/man2/sched_setparam.2.html
// This value is used when configuring the main loop to use SCHED_FIFO scheduling
// We use a midpoint RT priority to allow maximum flexibility to users
int const kSchedPriority = 50;

// rt_control: written by the update thread only, read by the reporting timer.
struct OverrunCounters
{
  std::atomic<std::uint64_t> events{0U};
  std::atomic<std::uint64_t> missed_cycles{0U};
  std::atomic<std::uint64_t> last_missed_cycles{0U};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";

  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name);

  const bool use_sim_time = cm->get_parameter_or("use_sim_time", false);

  const bool has_realtime = realtime_tools::has_realtime_kernel();
  const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", has_realtime);
  if (lock_memory) {
    const auto lock_result = realtime_tools::lock_memory();
    if (!lock_result.first) {
      RCLCPP_WARN(cm->get_logger(), "Unable to lock the memory: '%s'", lock_result.second.c_str());
    }
  }

  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());
  // rt_control: same parameter name and default as ros2_control Jazzy.
  const bool manage_overruns = cm->get_parameter_or<bool>("overruns.manage", true);
  RCLCPP_INFO(
    cm->get_logger(), "Overruns handling is : %s", manage_overruns ? "enabled" : "disabled");
  const int thread_priority = cm->get_parameter_or<int>("thread_priority", kSchedPriority);
  RCLCPP_INFO(
    cm->get_logger(), "Spawning %s RT thread with scheduler priority: %d", cm->get_name(),
    thread_priority);

  auto overruns = std::make_shared<OverrunCounters>();

  std::thread cm_thread([cm, thread_priority, use_sim_time, manage_overruns, overruns]() {
    rclcpp::Parameter cpu_affinity_param;
    if (cm->get_parameter("cpu_affinity", cpu_affinity_param)) {
      std::vector<int> cpus = {};
      if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        cpus = {static_cast<int>(cpu_affinity_param.as_int())};
      } else if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        const auto cpu_affinity_param_array = cpu_affinity_param.as_integer_array();
        std::for_each(
          cpu_affinity_param_array.begin(), cpu_affinity_param_array.end(),
          [&cpus](int cpu) { cpus.push_back(static_cast<int>(cpu)); });
      }
      const auto affinity_result = realtime_tools::set_current_thread_affinity(cpus);
      if (!affinity_result.first) {
        RCLCPP_WARN(
          cm->get_logger(), "Unable to set the CPU affinity : '%s'",
          affinity_result.second.c_str());
      }
    }

    if (!realtime_tools::configure_sched_fifo(thread_priority)) {
      RCLCPP_WARN(
        cm->get_logger(),
        "Could not enable FIFO RT scheduling policy: with error number <%i>(%s). See "
        "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
        "for details on how to enable realtime scheduling.",
        errno, strerror(errno));
    } else {
      RCLCPP_INFO(
        cm->get_logger(), "Successful set up FIFO RT scheduling policy with priority %i.",
        thread_priority);
    }

    // for calculating sleep time
    auto const period = std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());
    // rt_control: monotonic deadline (2.53.1 used system_clock, which also jumps with wall time).
    auto next_iteration_time = std::chrono::steady_clock::now();

    // for calculating the measured period of the loop
    rclcpp::Time previous_time = cm->now();

    while (rclcpp::ok()) {
      // calculate measured period
      auto const current_time = cm->now();
      auto const measured_period = current_time - previous_time;
      previous_time = current_time;

      // execute update loop
      cm->read(cm->now(), measured_period);
      cm->update(cm->now(), measured_period);
      cm->write(cm->now(), measured_period);

      // wait until we hit the end of the period
      if (use_sim_time) {
        next_iteration_time += period;
        cm->get_clock()->sleep_until(current_time + period);
      } else {
        // rt_control: skip missed ticks instead of catching up (Jazzy `overruns.manage`).
        const auto step = rt_control_bringup::next_deadline(
          next_iteration_time, std::chrono::steady_clock::now(), period, manage_overruns);
        next_iteration_time = step.deadline;
        if (step.missed_cycles > 0U) {
          overruns->events.fetch_add(1U, std::memory_order_relaxed);
          overruns->missed_cycles.fetch_add(step.missed_cycles, std::memory_order_relaxed);
          overruns->last_missed_cycles.store(step.missed_cycles, std::memory_order_relaxed);
        }
        std::this_thread::sleep_until(next_iteration_time);
      }
    }
  });

  // rt_control: report overruns outside the update thread, at most once per second.
  struct Reported
  {
    std::uint64_t events{0U};
    std::uint64_t missed{0U};
  };
  auto reported = std::make_shared<Reported>();
  auto overrun_report = cm->create_wall_timer(1s, [cm, overruns, reported]() {
    const auto events = overruns->events.load(std::memory_order_relaxed);
    if (events == reported->events) {
      return;
    }
    const auto missed = overruns->missed_cycles.load(std::memory_order_relaxed);
    RCLCPP_WARN(
      cm->get_logger(),
      "Overrun detected! %llu overrun(s) skipped %llu missed cycle(s) instead of running them "
      "back to back (last overrun: %llu cycle(s); total skipped since start: %llu).",
      static_cast<unsigned long long>(events - reported->events),
      static_cast<unsigned long long>(missed - reported->missed),
      static_cast<unsigned long long>(overruns->last_missed_cycles.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(missed));
    reported->events = events;
    reported->missed = missed;
  });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
