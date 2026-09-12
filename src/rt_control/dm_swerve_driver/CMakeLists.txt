cmake_minimum_required(VERSION 3.16)
project(dm_swerve_driver LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
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
option(DM_SWERVE_ENABLE_IGH "Build the real IgH 1.6 EtherCAT adapter" OFF)

function(dm_swerve_configure_target target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(${target}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  if(DM_SWERVE_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${target} PRIVATE --coverage -O0 -g)
    target_link_options(${target} PUBLIC --coverage)
  endif()
endfunction()

add_library(dm_swerve_math
  src/odometry_covariance.cpp
  src/params.cpp
  src/swerve_kinematics.cpp
  src/swerve_least_squares.cpp
  src/swerve_odometry.cpp
  src/swerve_setpoint.cpp
  src/swerve_steering.cpp
)
dm_swerve_configure_target(dm_swerve_math)

add_library(dm_swerve_can
  src/can_transport.cpp
  src/socketcan_interface.cpp
)
dm_swerve_configure_target(dm_swerve_can)

add_library(dm_swerve_kinco
  src/atomic_text_file.cpp
  src/canopen_encoder.cpp
  src/canopen_encoder_setup.cpp
  src/encoder_snapshot_store.cpp
  src/external_steering_encoder.cpp
  src/kinco_backend.cpp
  src/kinco_ds402.cpp
  src/kinco_params.cpp
  src/kinco_units.cpp
)
dm_swerve_configure_target(dm_swerve_kinco)
target_link_libraries(dm_swerve_kinco PUBLIC dm_swerve_can dm_swerve_math)
if(DM_SWERVE_ENABLE_IGH)
  find_path(IGH_INCLUDE_DIR ecrt.h REQUIRED)
  find_library(IGH_LIBRARY ethercat REQUIRED)
  target_sources(dm_swerve_kinco PRIVATE src/igh_ethercat_bus.cpp)
  target_include_directories(dm_swerve_kinco SYSTEM PRIVATE ${IGH_INCLUDE_DIR})
  target_link_libraries(dm_swerve_kinco PUBLIC ${IGH_LIBRARY})
else()
  target_sources(dm_swerve_kinco PRIVATE src/igh_unavailable.cpp)
endif()

add_library(dm_swerve_control
  src/kinco_calibration.cpp
  src/kinco_control_cycle.cpp
  src/kinco_control_loop.cpp
  src/kinco_safety_monitor.cpp
)
dm_swerve_configure_target(dm_swerve_control)
target_link_libraries(dm_swerve_control PUBLIC dm_swerve_kinco dm_swerve_math)

add_library(dm_swerve_ros
  src/imu_validation.cpp
  src/kinco_diagnostics.cpp
  src/kinco_ros_params.cpp
  src/ros_output.cpp
  src/ros_params.cpp
  src/swerve_driver_node.cpp
)
dm_swerve_configure_target(dm_swerve_ros)
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

add_executable(swerve_driver_node src/main.cpp)
target_compile_features(swerve_driver_node PRIVATE cxx_std_17)
target_compile_options(swerve_driver_node PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(swerve_driver_node dm_swerve_ros)
ament_target_dependencies(swerve_driver_node rclcpp rclcpp_lifecycle)

install(TARGETS
  dm_swerve_math
  dm_swerve_can
  dm_swerve_kinco
  dm_swerve_control
  dm_swerve_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS swerve_driver_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config doc launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_add_gtest(test_socketcan_vcan test/test_socketcan_vcan.cpp)
  target_link_libraries(test_socketcan_vcan dm_swerve_can)

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics dm_swerve_math)

  ament_add_gtest(test_odometry test/test_odometry.cpp)
  target_link_libraries(test_odometry dm_swerve_math)

  ament_add_gtest(test_odometry_covariance test/test_odometry_covariance.cpp)
  target_link_libraries(test_odometry_covariance dm_swerve_math)

  ament_add_gtest(test_params test/test_params.cpp)
  target_link_libraries(test_params dm_swerve_math)

  ament_add_gtest(test_safety test/test_safety.cpp)
  target_link_libraries(test_safety dm_swerve_control)

  ament_add_gtest(test_kinco_ds402 test/test_kinco_ds402.cpp)
  target_link_libraries(test_kinco_ds402 dm_swerve_kinco)

  ament_add_gtest(test_kinco_units test/test_kinco_units.cpp)
  target_link_libraries(test_kinco_units dm_swerve_kinco)

  ament_add_gtest(test_external_steering_encoder test/test_external_steering_encoder.cpp)
  target_link_libraries(test_external_steering_encoder dm_swerve_kinco)

  ament_add_gtest(test_encoder_snapshot_store test/test_encoder_snapshot_store.cpp)
  target_link_libraries(test_encoder_snapshot_store dm_swerve_kinco)

  ament_add_gtest(test_kinco_backend test/test_kinco_backend.cpp)
  target_link_libraries(test_kinco_backend dm_swerve_kinco)

  ament_add_gtest(test_canopen_encoder test/test_canopen_encoder.cpp)
  target_link_libraries(test_canopen_encoder dm_swerve_kinco)

  ament_add_gtest(test_kinco_params test/test_kinco_params.cpp)
  target_link_libraries(test_kinco_params dm_swerve_kinco)

  ament_add_gtest(test_kinco_runtime test/test_kinco_runtime.cpp)
  target_link_libraries(test_kinco_runtime dm_swerve_control)

  ament_add_gtest(test_ros_params test/test_ros_params.cpp)
  target_link_libraries(test_ros_params dm_swerve_ros)
  ament_target_dependencies(test_ros_params rclcpp rclcpp_lifecycle)

  ament_add_gtest(test_diagnostics test/test_diagnostics.cpp)
  target_link_libraries(test_diagnostics dm_swerve_ros)
  ament_target_dependencies(test_diagnostics diagnostic_msgs)

  ament_add_gtest(test_imu_validation test/test_imu_validation.cpp)
  target_link_libraries(test_imu_validation dm_swerve_ros)
  ament_target_dependencies(test_imu_validation sensor_msgs)

  add_test(
    NAME test_launch_description
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_description.py
      ${CMAKE_CURRENT_SOURCE_DIR}/launch/swerve_driver.launch.py
  )
  set_tests_properties(test_launch_description PROPERTIES LABELS "launch")
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
