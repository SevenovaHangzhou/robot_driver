cmake_minimum_required(VERSION 3.8)
project(rt_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robot_rt_control_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointControlModeResult.msg"
  "msg/PlcIoState.msg"
  "srv/RtEnable.srv"
  DEPENDENCIES robot_rt_control_interfaces std_msgs unique_identifier_msgs
)

ament_package()
