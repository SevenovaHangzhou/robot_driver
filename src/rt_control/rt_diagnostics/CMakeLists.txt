cmake_minimum_required(VERSION 3.8)
project(rt_diagnostics)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(robot_interfaces_qos REQUIRED)
find_package(rt_control_semantic_components REQUIRED)

add_library(
  rt_diagnostics_topology
  src/canopen_native_diagnostic.cpp
  src/diagnostic_summary.cpp
  src/diagnostics_topology.cpp
  src/dynamic_state_snapshot.cpp
  src/ethercat_master_diagnostic.cpp
)
target_compile_features(rt_diagnostics_topology PUBLIC cxx_std_17)
target_compile_options(rt_diagnostics_topology PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_include_directories(
  rt_diagnostics_topology PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rt_diagnostics_topology control_msgs)

add_executable(rt_diagnostics_node src/rt_diagnostics_node.cpp)
target_compile_features(rt_diagnostics_node PUBLIC cxx_std_17)
target_compile_options(rt_diagnostics_node PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_link_libraries(rt_diagnostics_node rt_diagnostics_topology)
ament_target_dependencies(
  rt_diagnostics_node
  control_msgs
  diagnostic_msgs
  rclcpp
  rcl_interfaces
  robot_interfaces_qos
  rt_control_semantic_components
)

install(
  TARGETS rt_diagnostics_node rt_diagnostics_topology
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_diagnostics_topology test/test_diagnostics_topology.cpp)
  target_link_libraries(test_diagnostics_topology rt_diagnostics_topology)
  ament_add_gtest(test_dynamic_state_snapshot test/test_dynamic_state_snapshot.cpp)
  target_link_libraries(test_dynamic_state_snapshot rt_diagnostics_topology)
  ament_add_gtest(test_diagnostic_summary test/test_diagnostic_summary.cpp)
  target_link_libraries(test_diagnostic_summary rt_diagnostics_topology)
  ament_target_dependencies(test_diagnostic_summary diagnostic_msgs)
  ament_add_gtest(
    test_canopen_native_diagnostic
    test/test_canopen_native_diagnostic.cpp
  )
  target_link_libraries(test_canopen_native_diagnostic rt_diagnostics_topology)
  ament_add_gtest(
    test_ethercat_master_diagnostic
    test/test_ethercat_master_diagnostic.cpp
  )
  target_link_libraries(test_ethercat_master_diagnostic rt_diagnostics_topology)
endif()

ament_package()
