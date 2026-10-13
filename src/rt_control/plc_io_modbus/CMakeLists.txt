cmake_minimum_required(VERSION 3.8)
project(plc_io_modbus)
# Uniform warnings for all owned targets, including ament test targets.
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rt_control_interfaces REQUIRED)
find_package(robot_interfaces_qos REQUIRED)
find_package(robot_rt_control_interfaces REQUIRED)
find_package(robot_system_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VacuumSensorState.msg"
  DEPENDENCIES std_msgs
)

add_library(modbus_client STATIC src/modbus_client.cpp)
target_compile_features(modbus_client PUBLIC cxx_std_17)
target_include_directories(modbus_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS modbus_client EXPORT export_modbus_client
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_modbus_client HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

add_executable(plc_io_modbus_node src/node.cpp)
set_target_properties(plc_io_modbus_node PROPERTIES OUTPUT_NAME plc_io_modbus)
target_compile_features(plc_io_modbus_node PRIVATE cxx_std_17)
target_compile_options(plc_io_modbus_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  plc_io_modbus_node rclcpp rt_control_interfaces robot_interfaces_qos
  robot_rt_control_interfaces robot_system_interfaces std_msgs unique_identifier_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(plc_io_modbus_node "${cpp_typesupport_target}" modbus_client)
install(TARGETS plc_io_modbus_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_modbus_client test/test_modbus_client.cpp)
  target_link_libraries(test_modbus_client modbus_client)
  # No sockets or devices: all client IO and address resolution use test doubles.
  set_property(TARGET test_modbus_client APPEND_STRING PROPERTY LINK_FLAGS
    " -Wl,--wrap=send,--wrap=recv,--wrap=poll,--wrap=socket,--wrap=connect,--wrap=close,--wrap=getaddrinfo,--wrap=freeaddrinfo")
endif()
ament_package()
