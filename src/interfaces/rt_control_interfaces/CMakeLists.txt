cmake_minimum_required(VERSION 3.8)
project(rt_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(robot_rt_control_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointControlModeResult.msg"
  "msg/PlcIoState.msg"
  "srv/SetDigitalOutput.srv"
  "srv/RtEnable.srv"
  DEPENDENCIES builtin_interfaces robot_rt_control_interfaces std_msgs unique_identifier_msgs
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)

ament_package()
