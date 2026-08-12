cmake_minimum_required(VERSION 3.8)
project(robot_rt_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_system_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyState.msg"
  "msg/VacuumChannelFeedback.msg"
  "msg/VacuumChannelState.msg"
  "msg/VacuumChannelResult.msg"
  "msg/VacuumState.msg"
  "srv/SetControlEnabled.srv"
  "srv/SetPumpEnabled.srv"
  "action/VacuumGrip.action"
  DEPENDENCIES std_msgs robot_system_interfaces
)

ament_package()
