cmake_minimum_required(VERSION 3.8)
project(robot_hw_can)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(damiao_protocol SHARED
  src/head_state_machine.cpp
  src/parameter_tool.cpp
  src/protocol.cpp
  src/socketcan.cpp)
target_compile_features(damiao_protocol PUBLIC cxx_std_17)
target_compile_options(damiao_protocol PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_include_directories(damiao_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(damiao_system SHARED src/damiao_system.cpp)
target_compile_features(damiao_system PUBLIC cxx_std_17)
target_compile_options(damiao_system PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_include_directories(damiao_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(damiao_system damiao_protocol)
ament_target_dependencies(damiao_system hardware_interface pluginlib rclcpp)
pluginlib_export_plugin_description_file(hardware_interface robot_hw_can_plugins.xml)

add_executable(damiao_parameter_tool src/parameter_tool_main.cpp)
target_compile_features(damiao_parameter_tool PUBLIC cxx_std_17)
target_compile_options(damiao_parameter_tool PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(damiao_parameter_tool damiao_protocol)

install(TARGETS damiao_protocol damiao_system
  EXPORT export_robot_hw_can
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS damiao_parameter_tool DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(FILES robot_hw_can_plugins.xml README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol damiao_protocol)
  ament_add_gtest(test_damiao_system test/test_damiao_system.cpp)
  target_link_libraries(test_damiao_system damiao_system)
  ament_target_dependencies(test_damiao_system hardware_interface rclcpp)
  ament_add_gtest(test_head_state_machine test/test_head_state_machine.cpp)
  target_link_libraries(test_head_state_machine damiao_protocol)
  ament_add_gtest(test_parameter_tool test/test_parameter_tool.cpp)
  target_link_libraries(test_parameter_tool damiao_protocol)
  ament_add_pytest_test(test_config test/test_config.py)
endif()

ament_export_targets(export_robot_hw_can HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp)
ament_package()
