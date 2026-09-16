cmake_minimum_required(VERSION 3.8)
project(plc_io_modbus)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rt_control_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VacuumSensorState.msg"
  DEPENDENCIES std_msgs
)

add_executable(plc_io_modbus_node src/node.cpp)
set_target_properties(plc_io_modbus_node PROPERTIES OUTPUT_NAME plc_io_modbus)
target_compile_features(plc_io_modbus_node PRIVATE cxx_std_17)
target_compile_options(plc_io_modbus_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(plc_io_modbus_node rclcpp rt_control_interfaces std_msgs std_srvs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(plc_io_modbus_node "${cpp_typesupport_target}")
install(TARGETS plc_io_modbus_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
