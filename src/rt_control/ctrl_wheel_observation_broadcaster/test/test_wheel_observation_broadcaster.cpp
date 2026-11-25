#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ctrl_wheel_observation_broadcaster/wheel_observation_broadcaster.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/observation_meta.hpp"

namespace
{
using Broadcaster = ctrl_wheel_observation_broadcaster::WheelObservationBroadcaster;
using CallbackReturn = controller_interface::CallbackReturn;
using Message = robot_rt_control_interfaces::msg::WheelObservationArray;
using Channel = robot_rt_control_interfaces::msg::WheelFeedbackChannel;
using Wheel = robot_rt_control_interfaces::msg::WheelObservation;
using Meta = robot_rt_control_interfaces::msg::ObservationMeta;

// A deliberately permuted wiring fixture (FR, RR, FL, RL) exercises quadrant sorting.
std::vector<rclcpp::Parameter> parameters()
{
  return {
    {"steering_joints", std::vector<std::string>{"s1", "s2", "s3", "s4"}},
    {"drive_joints", std::vector<std::string>{"w1", "w2", "w3", "w4"}},
    {"steering_encoders", std::vector<std::string>{"e1", "e2", "e3", "e4"}},
    {"module_x", std::vector<double>{0.305, -0.305, 0.305, -0.305}},
    {"module_y", std::vector<double>{-0.305, -0.305, 0.305, 0.305}},
    {"wheel_radius", std::vector<double>(4, 0.1)},
    {"steering_to_yaw_sign", std::vector<double>(4, -1.0)},
    {"feedback_timeout", 0.2},
    {"publish_rate", 100.0},
    {"chassis_calibration_id", "swerve_chassis:sha256:0123456789abcdef"},
    {"encoder_calibration_id", "swerve_encoders:sha256:fedcba9876543210"},
    {"startup_id", "chassis_only-test"},
  };
}

class WheelObservationBroadcasterTest : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  virtual std::vector<rclcpp::Parameter> configuration() { return parameters(); }

  void SetUp() override
  {
    ASSERT_EQ(broadcaster.init("wheel_observation_test"), controller_interface::return_type::OK);
    broadcaster.get_node()->set_parameters(configuration());
    ASSERT_EQ(broadcaster.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    names = broadcaster.state_interface_configuration().names;
    ASSERT_EQ(names.size(), 44U);
    values.assign(names.size(), 0.0);
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto separator = names[i].find_last_of('/');
      handles.emplace_back(
        names[i].substr(0, separator), names[i].substr(separator + 1), &values[i]);
    }
    std::vector<hardware_interface::LoanedStateInterface> loaned;
    for (auto & handle : handles) {
      loaned.emplace_back(handle);
    }
    broadcaster.assign_interfaces({}, std::move(loaned));
    for (std::size_t m = 1; m <= 4; ++m) {
      set("s" + std::to_string(m) + "/status_word", 0x0040);
      set("w" + std::to_string(m) + "/status_word", 0x0040);
      set("e" + std::to_string(m) + "/feedback_age_ms", 1.0);
    }
    ASSERT_EQ(broadcaster.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    client = std::make_shared<rclcpp::Node>("wheel_observation_client");
    subscription = client->create_subscription<Message>(
      "/rt/base/wheel_observations", robot_interfaces_qos::fast_state(),
      [this](Message::SharedPtr message) {
        received = message;
        ++published;
      });
    executor.add_node(client);
    executor.add_node(broadcaster.get_node()->get_node_base_interface());
  }

  void set(const std::string & name, double value)
  {
    const auto found = std::find(names.begin(), names.end(), name);
    ASSERT_NE(found, names.end()) << name;
    values[static_cast<std::size_t>(found - names.begin())] = value;
  }

  // One 1 ms cycle; every drive delivers a new PDO cycle.
  void cycle(std::int64_t time_ns)
  {
    sequence += 1.0;
    for (std::size_t m = 1; m <= 4; ++m) {
      set("s" + std::to_string(m) + "/feedback_sequence", sequence);
      set("w" + std::to_string(m) + "/feedback_sequence", sequence);
    }
    broadcaster.update(rclcpp::Time(time_ns), rclcpp::Duration::from_seconds(0.001));
  }

  // Runs `cycles` 1 ms cycles from `time_ns`; returns the last message published.
  Message::SharedPtr run(std::int64_t time_ns, int cycles = 1)
  {
    received.reset();
    for (int i = 0; i < cycles; ++i) {
      before_cycle(i);
      cycle(time_ns + i * 1000000);
      wait();
    }
    return received;
  }

  void wait()
  {
    const auto expected = published + 1U;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
    while (published < expected && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(std::chrono::milliseconds(2));
    }
  }

  std::function<void(int)> before_cycle = [](int) {};
  std::size_t published{0U};

  Broadcaster broadcaster;
  std::vector<std::string> names;
  std::vector<double> values;
  std::vector<hardware_interface::StateInterface> handles;
  rclcpp::Node::SharedPtr client;
  rclcpp::Subscription<Message>::SharedPtr subscription;
  rclcpp::executors::SingleThreadedExecutor executor;
  Message::SharedPtr received;
  double sequence{0.0};
};

class OrderedWheelObservationTest : public WheelObservationBroadcasterTest
{
protected:
  std::vector<rclcpp::Parameter> configuration() override
  {
    auto result = parameters();
    result[3] = {"module_x", std::vector<double>{0.305, 0.305, -0.305, -0.305}};
    result[4] = {"module_y", std::vector<double>{0.305, -0.305, 0.305, -0.305}};
    return result;
  }
};

TEST_F(OrderedWheelObservationTest, AllFourCornersKeepTheirMotorAndEncoderSamples)
{
  for (int module = 1; module <= 4; ++module) {
    const auto suffix = std::to_string(module);
    set("s" + suffix + "/position", 0.1 * module);
    set("e" + suffix + "/position", -0.09 * module);
    set("w" + suffix + "/position", 10.0 * module);
    set("w" + suffix + "/velocity", 0.2 * module);
  }
  const auto message = run(1000000000);
  ASSERT_TRUE(message);
  for (std::size_t slot = 0; slot < 4; ++slot) {
    const auto & wheel = message->wheels[slot];
    const auto module = static_cast<double>(slot + 1);
    EXPECT_EQ(wheel.wheel_id, slot);
    EXPECT_DOUBLE_EQ(wheel.steering_axis_x, slot < 2 ? 0.305 : -0.305);
    EXPECT_DOUBLE_EQ(wheel.steering_axis_y, slot % 2 == 0 ? 0.305 : -0.305);
    EXPECT_DOUBLE_EQ(wheel.drive_position, 10.0 * module);
    EXPECT_DOUBLE_EQ(wheel.drive_velocity, 0.2 * module);
    EXPECT_DOUBLE_EQ(wheel.steering_motor_angle, -0.1 * module);
    EXPECT_DOUBLE_EQ(wheel.steering_encoder_angle, -0.09 * module);
    EXPECT_TRUE(wheel.drive.valid);
    EXPECT_TRUE(wheel.steering_motor.valid);
    EXPECT_TRUE(wheel.steering_encoder.valid);
  }
}

TEST_F(WheelObservationBroadcasterTest, OrdersWheelsByLocationAndKeepsSourcesSeparate)
{
  set("s1/position", 0.5);  // module_1 is front-right
  set("e1/position", -0.49);
  set("w1/position", 12.0);
  set("w1/velocity", -3.0);
  const auto message = run(1000000000);
  ASSERT_TRUE(message);
  EXPECT_EQ(message->header.frame_id, "base_footprint");
  EXPECT_EQ(
    message->calibration_id,
    "swerve_chassis:sha256:0123456789abcdef;swerve_encoders:sha256:fedcba9876543210");
  EXPECT_EQ(message->session_id, "chassis_only-test:000001");
  const std::array<std::uint8_t, 4> ids{
    Wheel::FRONT_LEFT, Wheel::FRONT_RIGHT, Wheel::REAR_LEFT, Wheel::REAR_RIGHT};
  for (std::size_t slot = 0; slot < 4; ++slot) {
    EXPECT_EQ(message->wheels[slot].wheel_id, ids[slot]);
  }
  const auto & front_right = message->wheels[Wheel::FRONT_RIGHT];
  EXPECT_DOUBLE_EQ(front_right.steering_axis_x, 0.305);
  EXPECT_DOUBLE_EQ(front_right.steering_axis_y, -0.305);
  EXPECT_DOUBLE_EQ(front_right.steering_motor_angle, -0.5);  // heading = -q
  EXPECT_DOUBLE_EQ(front_right.steering_encoder_angle, -0.49);
  EXPECT_DOUBLE_EQ(front_right.drive_position, 12.0);
  EXPECT_DOUBLE_EQ(front_right.drive_velocity, -3.0);
  EXPECT_TRUE(front_right.drive.valid);
  EXPECT_EQ(front_right.drive.time_source, Meta::TIME_SOURCE_READ_COMPLETE);
  EXPECT_EQ(front_right.steering_encoder.time_source, Meta::TIME_SOURCE_HOST_RECEIVE);
}

TEST_F(WheelObservationBroadcasterTest, EncoderSampleCountsOnlyNewFrames)
{
  // One encoder frame, received 1 ms before the first read, ages 1 ms per cycle; the drives
  // deliver a new PDO cycle every cycle.
  before_cycle = [this](int i) { set("e1/feedback_age_ms", 1.0 + i); };
  const auto message = run(1000000000, 25);
  ASSERT_TRUE(message);
  ASSERT_GT(message->sequence, 1U);
  const auto & wheel = message->wheels[Wheel::FRONT_RIGHT];
  EXPECT_TRUE(wheel.steering_encoder.valid);
  EXPECT_EQ(wheel.steering_encoder.sample_sequence, 1U);
  EXPECT_EQ(wheel.steering_encoder.sample_stamp.sec, 0);
  EXPECT_EQ(wheel.steering_encoder.sample_stamp.nanosec, 999000000U);
  EXPECT_GT(wheel.drive.sample_sequence, 10U);
  // A newer frame lowers the age and counts once.
  before_cycle = [this](int) { set("e1/feedback_age_ms", 0.5); };
  const auto next = run(1100000000, 11);
  ASSERT_TRUE(next);
  EXPECT_EQ(next->wheels[Wheel::FRONT_RIGHT].steering_encoder.sample_sequence, 2U);
}

TEST_F(WheelObservationBroadcasterTest, StaleFaultAndTimeRollback)
{
  set("w1/feedback_age_ms", 500.0);
  set("s1/status_word", 0x0008);  // CiA402 fault
  auto message = run(1000000000);
  ASSERT_TRUE(message);
  EXPECT_EQ(message->wheels[Wheel::FRONT_RIGHT].drive.reason, Channel::REASON_STALE);
  EXPECT_EQ(
    message->wheels[Wheel::FRONT_RIGHT].steering_motor.reason, Channel::REASON_DEVICE_FAULT);
  EXPECT_TRUE(message->wheels[Wheel::FRONT_LEFT].drive.valid);
  const auto first = message->sequence;
  message = run(500000000);  // ROS time stepped back
  ASSERT_TRUE(message);
  EXPECT_EQ(message->session_id, "chassis_only-test:000002");
  EXPECT_LE(message->sequence, first);
}

TEST_F(WheelObservationBroadcasterTest, RejectsTwoModulesInOneQuadrant)
{
  Broadcaster other;
  ASSERT_EQ(other.init("wheel_observation_bad"), controller_interface::return_type::OK);
  auto overlapping = parameters();
  overlapping[4] = {"module_y", std::vector<double>{-0.305, -0.305, 0.305, -0.305}};
  other.get_node()->set_parameters(overlapping);
  EXPECT_EQ(other.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}
}  // namespace
