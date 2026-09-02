cmake_minimum_required(VERSION 3.16)
project(dm_swerve_driver LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

option(DM_SWERVE_ENABLE_COVERAGE "Build dm_swerve_driver with GCC coverage" OFF)

add_library(dm_swerve_protocol
  src/bringup_audit.cpp
  src/dm_frame_codec.cpp
  src/fake_motor_model.cpp
  src/socketcan_interface.cpp
)
target_compile_features(dm_swerve_protocol PUBLIC cxx_std_17)
target_include_directories(dm_swerve_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(dm_swerve_protocol PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)

if(DM_SWERVE_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(dm_swerve_protocol PRIVATE --coverage -O0 -g)
  target_link_options(dm_swerve_protocol PUBLIC --coverage)
endif()

add_library(dm_swerve_ros
  src/diagnostics.cpp
  src/ros_output.cpp
  src/ros_params.cpp
  src/swerve_driver_node.cpp
)
target_compile_features(dm_swerve_ros PUBLIC cxx_std_17)
target_include_directories(dm_swerve_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(dm_swerve_ros dm_swerve_control)
ament_target_dependencies(dm_swerve_ros
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_compile_options(dm_swerve_ros PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)

if(DM_SWERVE_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(dm_swerve_ros PRIVATE --coverage -O0 -g)
  target_link_options(dm_swerve_ros PUBLIC --coverage)
endif()

add_executable(swerve_driver_node src/main.cpp)
target_compile_features(swerve_driver_node PRIVATE cxx_std_17)
target_link_libraries(swerve_driver_node dm_swerve_ros)
ament_target_dependencies(swerve_driver_node rclcpp rclcpp_lifecycle)
target_compile_options(swerve_driver_node PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)

add_library(dm_swerve_control
  src/can_transport.cpp
  src/control_cycle.cpp
  src/control_loop.cpp
  src/control_thread.cpp
  src/motor_startup.cpp
  src/params.cpp
  src/safety_monitor.cpp
)
target_compile_features(dm_swerve_control PUBLIC cxx_std_17)
target_include_directories(dm_swerve_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(dm_swerve_control PUBLIC dm_swerve_motor)
target_compile_options(dm_swerve_control PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)

if(DM_SWERVE_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(dm_swerve_control PRIVATE --coverage -O0 -g)
  target_link_options(dm_swerve_control PUBLIC --coverage)
endif()

add_library(dm_swerve_math
  src/swerve_kinematics.cpp
  src/swerve_odometry.cpp
)
target_compile_features(dm_swerve_math PUBLIC cxx_std_17)
target_include_directories(dm_swerve_math
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(dm_swerve_math PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)

if(DM_SWERVE_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(dm_swerve_math PRIVATE --coverage -O0 -g)
  target_link_options(dm_swerve_math PUBLIC --coverage)
endif()

add_library(dm_swerve_motor
  src/dm_motor.cpp
  src/feedback_router.cpp
  src/swerve_module.cpp
)
target_compile_features(dm_swerve_motor PUBLIC cxx_std_17)
target_include_directories(dm_swerve_motor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(dm_swerve_motor PUBLIC dm_swerve_protocol dm_swerve_math)
target_compile_options(dm_swerve_motor PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)

if(DM_SWERVE_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(dm_swerve_motor PRIVATE --coverage -O0 -g)
  target_link_options(dm_swerve_motor PUBLIC --coverage)
endif()

add_executable(fake_motor_sim tools/fake_motor_sim.cpp)
target_compile_features(fake_motor_sim PRIVATE cxx_std_17)
target_link_libraries(fake_motor_sim PRIVATE dm_swerve_protocol)
target_compile_options(fake_motor_sim PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)

add_executable(dm_swerve_bringup_check tools/bringup_check.cpp)
target_compile_features(dm_swerve_bringup_check PRIVATE cxx_std_17)
target_link_libraries(dm_swerve_bringup_check PRIVATE dm_swerve_protocol dm_swerve_motor)
target_compile_options(
  dm_swerve_bringup_check PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)

install(TARGETS
  dm_swerve_protocol dm_swerve_math dm_swerve_motor dm_swerve_control dm_swerve_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS fake_motor_sim swerve_driver_node dm_swerve_bringup_check
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config doc launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frame_codec test/test_frame_codec.cpp)
  target_link_libraries(test_frame_codec dm_swerve_protocol)

  ament_add_gtest(test_fake_motor_model test/test_fake_motor_model.cpp)
  target_link_libraries(test_fake_motor_model dm_swerve_protocol)

  ament_add_gtest(test_socketcan_vcan test/test_socketcan_vcan.cpp)
  target_link_libraries(test_socketcan_vcan dm_swerve_protocol)

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics dm_swerve_math)

  ament_add_gtest(test_odometry test/test_odometry.cpp)
  target_link_libraries(test_odometry dm_swerve_math)

  ament_add_gtest(test_motor_unwrap test/test_motor_unwrap.cpp)
  target_link_libraries(test_motor_unwrap dm_swerve_motor)

  ament_add_gtest(test_swerve_module test/test_swerve_module.cpp)
  target_link_libraries(test_swerve_module dm_swerve_motor)

  ament_add_gtest(test_feedback_router test/test_feedback_router.cpp)
  target_link_libraries(test_feedback_router dm_swerve_motor)

  ament_add_gtest(test_params test/test_params.cpp)
  target_link_libraries(test_params dm_swerve_control)

  ament_add_gtest(test_control_loop test/test_control_loop.cpp)
  target_link_libraries(test_control_loop dm_swerve_control dm_swerve_protocol)

  ament_add_gtest(test_safety test/test_safety.cpp)
  target_link_libraries(test_safety dm_swerve_control)

  ament_add_gtest(test_ros_params test/test_ros_params.cpp)
  target_link_libraries(test_ros_params dm_swerve_ros)
  ament_target_dependencies(test_ros_params rclcpp rclcpp_lifecycle)

  ament_add_gtest(test_swerve_driver_node test/test_swerve_driver_node.cpp)
  target_link_libraries(test_swerve_driver_node dm_swerve_ros dm_swerve_protocol)
  ament_target_dependencies(test_swerve_driver_node rclcpp rclcpp_lifecycle)

  ament_add_gtest(test_diagnostics test/test_diagnostics.cpp)
  target_link_libraries(test_diagnostics dm_swerve_ros)
  ament_target_dependencies(test_diagnostics diagnostic_msgs)

  ament_add_gtest(test_integration_fake_bus test/test_integration_fake_bus.cpp)
  target_link_libraries(test_integration_fake_bus dm_swerve_control dm_swerve_protocol)

  ament_add_gtest(test_bringup_audit test/test_bringup_audit.cpp)
  target_link_libraries(test_bringup_audit dm_swerve_protocol)

  add_test(
    NAME test_launch_description
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_description.py
      ${CMAKE_CURRENT_SOURCE_DIR}/launch/swerve_driver.launch.py
  )
  set_tests_properties(test_launch_description PROPERTIES LABELS "launch")

  add_test(
    NAME test_concurrency_contract
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_concurrency_contract.py
      ${CMAKE_CURRENT_SOURCE_DIR}/src/control_loop.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/control_thread.cpp
  )
  set_tests_properties(test_concurrency_contract PROPERTIES LABELS "concurrency")
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
ament_package()
