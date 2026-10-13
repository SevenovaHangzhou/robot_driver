cmake_minimum_required(VERSION 3.8)
project(modbus_tcp_rtu485)
# Uniform warnings for all owned targets, including ament test targets.
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces_qos REQUIRED)
find_package(robot_rt_control_interfaces REQUIRED)
find_package(robot_system_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

add_library(modbus_transport STATIC src/modbus_transport.cpp)
add_library(ultrasonic_decoder STATIC src/ultrasonic_protocol.cpp)
add_library(ultrasonic_ros_conversion STATIC src/ros_conversion.cpp)
foreach(target modbus_transport ultrasonic_decoder ultrasonic_ros_conversion)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endforeach()
target_link_libraries(ultrasonic_ros_conversion PUBLIC ultrasonic_decoder)
ament_target_dependencies(ultrasonic_ros_conversion PUBLIC diagnostic_msgs sensor_msgs)

add_executable(led_strip_node src/led_strip_node.cpp)
target_link_libraries(led_strip_node modbus_transport)
target_compile_features(led_strip_node PUBLIC cxx_std_17)
target_compile_options(led_strip_node PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
ament_target_dependencies(
  led_strip_node rclcpp robot_rt_control_interfaces robot_system_interfaces)

add_executable(ultrasonic_node src/ultrasonic_node.cpp)
target_link_libraries(ultrasonic_node modbus_transport ultrasonic_ros_conversion)
target_compile_features(ultrasonic_node PUBLIC cxx_std_17)
target_compile_options(ultrasonic_node PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
ament_target_dependencies(
  ultrasonic_node diagnostic_msgs rclcpp robot_interfaces_qos
  robot_rt_control_interfaces robot_system_interfaces sensor_msgs std_msgs
  unique_identifier_msgs)

install(TARGETS modbus_transport ultrasonic_decoder ultrasonic_ros_conversion
  EXPORT export_modbus_protocol
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_modbus_protocol HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(diagnostic_msgs sensor_msgs)

install(TARGETS led_strip_node ultrasonic_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ultrasonic_decoder test/test_ultrasonic_decoder.cpp)
  target_link_libraries(test_ultrasonic_decoder ultrasonic_decoder)
  ament_add_gtest(test_modbus_transport_library test/test_modbus_transport_library.cpp)
  target_link_libraries(test_modbus_transport_library modbus_transport)
  # Inject fake syscalls at link time; the library under test is the production archive.
  set_property(TARGET test_modbus_transport_library APPEND_STRING PROPERTY LINK_FLAGS
    " -Wl,--wrap=send,--wrap=recv,--wrap=poll")
  find_package(Threads REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_executable(test_modbus_transport test/test_modbus_transport.cpp)
  target_compile_features(test_modbus_transport PRIVATE cxx_std_17)
  target_link_libraries(test_modbus_transport Threads::Threads modbus_transport ultrasonic_ros_conversion)
  ament_target_dependencies(test_modbus_transport diagnostic_msgs sensor_msgs)
  add_test(NAME modbus_transport COMMAND test_modbus_transport)
  set_tests_properties(modbus_transport PROPERTIES TIMEOUT 20)
  add_test(NAME node_lifecycle
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_node_lifecycle.py
    $<TARGET_FILE:led_strip_node> ${CMAKE_CURRENT_SOURCE_DIR}/config/led_strip.yaml
    $<TARGET_FILE:ultrasonic_node> ${CMAKE_CURRENT_SOURCE_DIR}/config/ultrasonic.yaml)
  set_tests_properties(node_lifecycle PROPERTIES TIMEOUT 30)
endif()
ament_package()
