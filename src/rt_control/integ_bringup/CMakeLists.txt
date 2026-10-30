cmake_minimum_required(VERSION 3.8)
project(integ_bringup)
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

ament_python_install_package(${PROJECT_NAME})

# Controller manager node: ros2_control 2.53.1 ros2_control_node with Jazzy's overrun management
# (skip missed ticks instead of running them back to back). Same executable name as upstream.
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
add_executable(ros2_control_node src/ros2_control_node.cpp)
target_compile_features(ros2_control_node PUBLIC cxx_std_17)
target_include_directories(ros2_control_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(ros2_control_node controller_manager rclcpp realtime_tools)
install(TARGETS ros2_control_node DESTINATION lib/${PROJECT_NAME})

install(
  FILES
    launch/rt_control_module.launch.py
    launch/rt_control_ultrasonic.launch.py
    launch/rt_control_scope_runtime.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  PROGRAMS
    scripts/rt_control_start
    scripts/validate_machine_profile.py
    scripts/scope_hardware_start
    scripts/scope_shutdown
    scripts/scope_capabilities
    scripts/scope_operator_tuning
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_loop_timing test/test_control_loop_timing.cpp)
  target_compile_features(test_control_loop_timing PUBLIC cxx_std_17)
  target_include_directories(test_control_loop_timing PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ultrasonic_launch test/test_ultrasonic_launch.py)
  ament_add_pytest_test(test_scope_lifecycle test/test_scope_lifecycle.py)
  ament_add_pytest_test(test_operator_tuning test/test_operator_tuning.py)
endif()

ament_package()
