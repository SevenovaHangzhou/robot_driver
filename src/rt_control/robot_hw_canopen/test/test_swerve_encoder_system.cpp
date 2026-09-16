#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <utility>

#include "hardware_interface/hardware_info.hpp"
#include "robot_hw_canopen/swerve_encoder_system.hpp"

namespace robot_hw_canopen
{
namespace
{

[[nodiscard]] hardware_interface::HardwareInfo valid_info()
{
  hardware_interface::HardwareInfo info;
  info.name = "swerve_encoder_system";
  info.type = "system";
  info.hardware_class_type = "robot_hw_canopen/SwerveEncoderSystem";
  info.hardware_parameters = {
    {"bus_config", "/tmp/swerve-encoders.yml"},
    {"master_config", "/tmp/master.dcf"},
    {"master_bin", "/tmp/master.bin"},
    {"can_interface_name", "can0"},
  };
  const std::array<std::string, kSwerveEncoderCount> names{
    "front_left_steering_encoder", "front_right_steering_encoder",
    "rear_left_steering_encoder", "rear_right_steering_encoder"};
  for (std::size_t index{0U}; index < names.size(); ++index) {
    hardware_interface::ComponentInfo sensor;
    sensor.name = names[index];
    sensor.type = "sensor";
    sensor.state_interfaces = {
      hardware_interface::InterfaceInfo{"position"},
      hardware_interface::InterfaceInfo{"feedback_age_ms"}};
    sensor.parameters = {
      {"node_id", std::to_string(index + 1U)},
      {"counts_per_revolution", "10000"},
      {"ring_gear_teeth", "2"},
      {"pinion_gear_teeth", "1"},
      {"direction", index % 2U == 0U ? "1" : "-1"},
      {"installation_offset_rad", "0.1"},
    };
    info.sensors.push_back(std::move(sensor));
  }
  return info;
}

class InspectableSwerveEncoderSystem : public SwerveEncoderSystem
{
public:
  void inject(
    ros2_canopen::COData data, std::uint8_t id,
    std::chrono::steady_clock::time_point received_at)
  {
    on_rpdo_received(data, id, received_at);
  }
};

TEST(SwerveEncoderSystemTest, ExportsExactlyFourReadOnlyEncoderPairs)
{
  SwerveEncoderSystem system;
  ASSERT_EQ(system.on_init(valid_info()), hardware_interface::CallbackReturn::SUCCESS);

  const auto states = system.export_state_interfaces();
  const auto commands = system.export_command_interfaces();
  std::set<std::string> names;
  for (const auto & state : states) {
    names.insert(state.get_name());
  }

  EXPECT_TRUE(commands.empty());
  EXPECT_EQ(states.size(), 2U * kSwerveEncoderCount);
  EXPECT_EQ(names, (std::set<std::string>{
    "front_left_steering_encoder/position",
    "front_left_steering_encoder/feedback_age_ms",
    "front_right_steering_encoder/position",
    "front_right_steering_encoder/feedback_age_ms",
    "rear_left_steering_encoder/position",
    "rear_left_steering_encoder/feedback_age_ms",
    "rear_right_steering_encoder/position",
    "rear_right_steering_encoder/feedback_age_ms"}));
  for (std::size_t index{0U}; index < states.size(); index += 2U) {
    EXPECT_TRUE(std::isnan(states[index].get_value()));
    EXPECT_TRUE(std::isinf(states[index + 1U].get_value()));
  }
}

TEST(SwerveEncoderSystemTest, RejectsWrongSensorCountInterfacesAndParameters)
{
  auto info = valid_info();
  info.sensors.pop_back();
  SwerveEncoderSystem system;
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);

  info = valid_info();
  info.sensors[0].state_interfaces[1].name = "velocity";
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);

  info = valid_info();
  info.sensors[0].parameters.erase("ring_gear_teeth");
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);

  info = valid_info();
  info.sensors[1].parameters["node_id"] = "1";
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
}

TEST(SwerveEncoderSystemTest, RpdoHookUpdatesObjectValueAndReceiveAge)
{
  InspectableSwerveEncoderSystem system;
  ASSERT_EQ(system.on_init(valid_info()), hardware_interface::CallbackReturn::SUCCESS);
  auto states = system.export_state_interfaces();

  ros2_canopen::COData data;
  data.index_ = kSwerveEncoderPositionIndex;
  data.subindex_ = kSwerveEncoderPositionSubindex;
  data.data_ = 1000U;
  system.inject(data, 3U, std::chrono::steady_clock::now());

  ASSERT_EQ(
    system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.001)),
    hardware_interface::return_type::OK);
  EXPECT_NEAR(states[4].get_value(), 0.1 * 3.14159265358979323846 - 0.1, 1e-12);
  EXPECT_TRUE(std::isfinite(states[5].get_value()));
  EXPECT_GE(states[5].get_value(), 0.0);
}

TEST(SwerveEncoderSystemTest, WrongRpdoObjectDoesNotFabricateFeedback)
{
  InspectableSwerveEncoderSystem system;
  ASSERT_EQ(system.on_init(valid_info()), hardware_interface::CallbackReturn::SUCCESS);
  auto states = system.export_state_interfaces();

  ros2_canopen::COData data;
  data.index_ = 0x6501U;
  data.subindex_ = 0U;
  data.data_ = 1000U;
  system.inject(data, 1U, std::chrono::steady_clock::now());
  ASSERT_EQ(
    system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.001)),
    hardware_interface::return_type::OK);

  EXPECT_TRUE(std::isnan(states[0].get_value()));
  EXPECT_TRUE(std::isinf(states[1].get_value()));
}

}  // namespace
}  // namespace robot_hw_canopen
