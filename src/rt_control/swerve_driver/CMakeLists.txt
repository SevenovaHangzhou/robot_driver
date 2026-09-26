cmake_minimum_required(VERSION 3.16)
project(swerve_driver LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
set(dependencies controller_interface hardware_interface diagnostic_msgs geometry_msgs
  nav_msgs sensor_msgs pluginlib rclcpp realtime_tools robot_interfaces_qos rt_control_semantic_components)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

option(SWERVE_ENABLE_COVERAGE "Instrument swerve targets for GCC coverage" OFF)
add_library(swerve_math SHARED
  src/swerve_kinematics.cpp src/swerve_odometry.cpp src/swerve_setpoint.cpp
  src/odometry_covariance.cpp src/control_core.cpp
  src/relative_move_planner.cpp src/relative_move_session.cpp src/chassis_mode_handoff.cpp)
add_library(swerve_controller SHARED src/swerve_controller.cpp src/chassis_runtime.cpp)
target_link_libraries(swerve_controller PUBLIC swerve_math)
ament_target_dependencies(swerve_controller PUBLIC ${dependencies})

find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rt_control_interfaces REQUIRED)
ament_target_dependencies(swerve_controller PUBLIC rclcpp_action rt_control_interfaces)
# A separate executable/library with no hardware/plugin dependency. Never loaded by production launch.
add_library(swerve_relative_move_mock SHARED src/relative_move_mock_node.cpp)
target_link_libraries(swerve_relative_move_mock PUBLIC swerve_math)
ament_target_dependencies(swerve_relative_move_mock PUBLIC rclcpp rclcpp_action rcl_interfaces geometry_msgs rt_control_interfaces)
add_executable(chassis_relative_move_mock src/relative_move_mock_main.cpp)
target_link_libraries(chassis_relative_move_mock swerve_relative_move_mock)
install(TARGETS swerve_relative_move_mock chassis_relative_move_mock
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES test/fixtures/relative_move_mock.yaml DESTINATION share/${PROJECT_NAME}/test)

foreach(target swerve_math swerve_controller swerve_relative_move_mock)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  if(SWERVE_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${target} PRIVATE --coverage -O0 -g)
    target_link_options(${target} PUBLIC --coverage)
  endif()
endforeach()

pluginlib_export_plugin_description_file(controller_interface swerve_plugins.xml)
install(TARGETS swerve_math swerve_controller EXPORT export_swerve_driver
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE NOTICE.md README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_swerve_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} rclcpp_action rcl_interfaces rt_control_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_relative_move_mock_process test/test_relative_move_mock_process.py
    TIMEOUT 40 ENV
    "SWERVE_RELATIVE_MOCK_BINARY=$<TARGET_FILE:chassis_relative_move_mock>"
    "SWERVE_RELATIVE_MOCK_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/relative_move_mock.yaml")
  find_package(controller_manager REQUIRED)
  find_package(robot_hw_ethercat REQUIRED)
  ament_add_gtest(test_cyclic_controller test/test_cyclic_controller.cpp TIMEOUT 90)
  target_link_libraries(test_cyclic_controller swerve_controller)
  ament_target_dependencies(test_cyclic_controller robot_hw_ethercat)
  target_compile_definitions(test_cyclic_controller PRIVATE
    KINCO_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/../robot_hw_ethercat/test/fixtures/kinco_cyclic_mode_mock.yaml")
  foreach(test kinematics odometry odometry_covariance control_core relative_move_planner relative_move_session chassis_mode_handoff)
    ament_add_gtest(test_${test} test/test_${test}.cpp)
    target_link_libraries(test_${test} swerve_math)
  endforeach()
  ament_add_gtest(test_relative_move_mock test/test_relative_move_mock.cpp TIMEOUT 120)
  target_link_libraries(test_relative_move_mock swerve_relative_move_mock)
  target_compile_definitions(test_relative_move_mock PRIVATE
    SWERVE_RELATIVE_MOCK_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/relative_move_mock.yaml")
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller swerve_controller)
  ament_add_gtest(test_controller_manager test/test_controller_manager.cpp)
  target_link_libraries(test_controller_manager swerve_controller)
  ament_target_dependencies(test_controller_manager controller_manager)
  target_compile_definitions(test_controller_manager PRIVATE
    SWERVE_MOCK_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/chassis.urdf")
endif()

ament_package()
