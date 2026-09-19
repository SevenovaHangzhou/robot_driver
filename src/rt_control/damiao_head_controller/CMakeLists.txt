cmake_minimum_required(VERSION 3.16)
project(damiao_head_controller LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces_qos REQUIRED)
find_package(robot_hw_can REQUIRED)
find_package(rt_control_interfaces REQUIRED)

add_library(head_manager_core SHARED src/head_manager_core.cpp)
target_compile_features(head_manager_core PUBLIC cxx_std_17)
target_compile_options(head_manager_core PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_include_directories(head_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(head_manager_core PUBLIC robot_hw_can)

add_library(damiao_head_manager SHARED src/head_manager_controller.cpp)
target_compile_features(damiao_head_manager PUBLIC cxx_std_17)
target_compile_options(damiao_head_manager PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_include_directories(damiao_head_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(damiao_head_manager PUBLIC head_manager_core)
ament_target_dependencies(damiao_head_manager PUBLIC
  controller_manager_msgs controller_interface diagnostic_msgs hardware_interface pluginlib rclcpp
  robot_interfaces_qos robot_hw_can rt_control_interfaces)
pluginlib_export_plugin_description_file(controller_interface damiao_head_controller_plugins.xml)

add_executable(head_disable_once src/head_disable_once.cpp)
target_compile_features(head_disable_once PUBLIC cxx_std_17)
target_compile_options(head_disable_once PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
ament_target_dependencies(head_disable_once rclcpp rt_control_interfaces)

install(TARGETS head_manager_core damiao_head_manager
  EXPORT export_damiao_head_controller
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS head_disable_once DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES damiao_head_controller_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_head_manager_core test/test_head_manager_core.cpp)
  target_link_libraries(test_head_manager_core head_manager_core)
  ament_add_gtest(test_head_manager_controller test/test_head_manager_controller.cpp)
  target_link_libraries(test_head_manager_controller damiao_head_manager)
  ament_target_dependencies(
    test_head_manager_controller controller_interface hardware_interface rclcpp)
endif()

ament_export_targets(export_damiao_head_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_manager_msgs controller_interface diagnostic_msgs hardware_interface pluginlib rclcpp
  robot_interfaces_qos robot_hw_can rt_control_interfaces)
ament_package()
