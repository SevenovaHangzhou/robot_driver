cmake_minimum_required(VERSION 3.8)
project(integ_bringup)
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

ament_python_install_package(${PROJECT_NAME})

# Controller manager node: the installed ros2_control 2.53.1 (Humble) ships ros2_control_node only
# as a binary, so its source is kept verbatim in upstream/ and one narrow patch adds Jazzy's overrun
# management (skip missed ticks instead of running them back to back). Same executable name.
find_package(controller_manager REQUIRED)
# The patched main() is 2.53.1's; it links the installed controller_manager library.
# 2.54.2 ships a byte-identical ros2_control_node.cpp (checked 2026-11-05).
if(NOT controller_manager_VERSION VERSION_EQUAL "2.53.1"
   AND NOT controller_manager_VERSION VERSION_EQUAL "2.54.2")
  message(FATAL_ERROR
    "controller_manager ${controller_manager_VERSION} installed; upstream/ros2_control_node.cpp and "
    "its patch are for 2.53.1/2.54.2. Re-import the upstream file and rebase the patch.")
endif()
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_program(PATCH_EXECUTABLE patch REQUIRED)
set(ROS2_CONTROL_NODE_UPSTREAM ${CMAKE_CURRENT_SOURCE_DIR}/upstream/ros2_control_node.cpp)
set(ROS2_CONTROL_NODE_PATCH ${CMAKE_CURRENT_SOURCE_DIR}/patches/ros2_control_node-overruns.patch)
set(ROS2_CONTROL_NODE_SOURCE ${CMAKE_CURRENT_BINARY_DIR}/patched/ros2_control_node.cpp)
add_custom_command(
  OUTPUT ${ROS2_CONTROL_NODE_SOURCE}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/patched
  COMMAND ${CMAKE_COMMAND} -E copy ${ROS2_CONTROL_NODE_UPSTREAM} ${ROS2_CONTROL_NODE_SOURCE}
  COMMAND ${PATCH_EXECUTABLE} --batch --forward --silent -p1
          -d ${CMAKE_CURRENT_BINARY_DIR}/patched -i ${ROS2_CONTROL_NODE_PATCH}
  DEPENDS ${ROS2_CONTROL_NODE_UPSTREAM} ${ROS2_CONTROL_NODE_PATCH}
  COMMENT "Applying the rt_control patch to ros2_control 2.53.1 ros2_control_node.cpp")
add_executable(ros2_control_node ${ROS2_CONTROL_NODE_SOURCE})
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
