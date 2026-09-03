cmake_minimum_required(VERSION 3.8)
project(rt_diagnostics)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces_qos REQUIRED)

add_executable(rt_diagnostics_node src/rt_diagnostics_node.cpp)
target_compile_features(rt_diagnostics_node PUBLIC cxx_std_17)
target_compile_options(rt_diagnostics_node PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
ament_target_dependencies(
  rt_diagnostics_node control_msgs diagnostic_msgs rclcpp robot_interfaces_qos
)

install(TARGETS rt_diagnostics_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_diagnostic_contract test/test_diagnostic_contract.py)
endif()

ament_package()
