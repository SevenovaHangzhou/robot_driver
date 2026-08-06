cmake_minimum_required(VERSION 3.8)
project(robot_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyState.msg"
  "msg/VacuumChannelFeedback.msg"
  "msg/VacuumChannelState.msg"
  "msg/VacuumChannelResult.msg"
  "msg/VacuumState.msg"
  "srv/SetControlEnabled.srv"
  "srv/SetPumpEnabled.srv"
  "action/VacuumGrip.action"
  DEPENDENCIES std_msgs
)

ament_package()
