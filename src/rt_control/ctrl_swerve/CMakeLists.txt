cmake_minimum_required(VERSION 3.16)
project(ctrl_swerve LANGUAGES C CXX)
# Uniform warnings for all owned targets, including ament test targets.
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)


find_package(ament_cmake REQUIRED)
find_package(ruckig REQUIRED)
set(dependencies controller_interface hardware_interface diagnostic_msgs diagnostic_updater geometry_msgs
  nav_msgs sensor_msgs std_msgs pluginlib rclcpp realtime_tools robot_interfaces_qos
  hw_semantic_components)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

option(SWERVE_ENABLE_COVERAGE "Instrument swerve targets for GCC coverage" OFF)
add_library(swerve_math SHARED
  src/swerve_kinematics.cpp src/swerve_odometry.cpp src/swerve_setpoint.cpp
  src/odometry_covariance.cpp src/control_core.cpp src/body_velocity_shaper.cpp)
target_link_libraries(swerve_math PUBLIC ruckig::ruckig)
add_library(swerve_controller SHARED src/swerve_controller.cpp src/navigation_feedback.cpp
  src/steering_velocity_observer.cpp)
target_link_libraries(swerve_controller PUBLIC swerve_math)
ament_target_dependencies(swerve_controller PUBLIC ${dependencies})

foreach(target swerve_math swerve_controller)
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
ament_export_dependencies(${dependencies} ruckig)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)
  foreach(test kinematics odometry odometry_covariance control_core)
    ament_add_gtest(test_${test} test/test_${test}.cpp)
    target_link_libraries(test_${test} swerve_math)
  endforeach()
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller swerve_controller)
  ament_add_gtest(test_navigation_feedback test/test_navigation_feedback.cpp)
  target_link_libraries(test_navigation_feedback swerve_controller)
  # Offline cross-repository proof; the supplied source stays outside this repository.
  set(FOLLOW_MPPI_REFERENCE_DIR "" CACHE PATH "FollowMPPI handoff reference_code directory")
  if(FOLLOW_MPPI_REFERENCE_DIR)
    find_package(tf2 REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    set(prediction "${FOLLOW_MPPI_REFERENCE_DIR}/navigation_prediction")
    set(production "${prediction}/third_party/production_swerve")
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME navigation_reference_integrity
      COMMAND ${Python3_EXECUTABLE} ${prediction}/test/test_production_snapshot.py)
    add_library(nav_compat_prediction STATIC
      ${prediction}/src/driver_prediction.cpp
      ${production}/src/control_core.cpp
      ${production}/src/body_velocity_shaper.cpp
      ${production}/src/swerve_kinematics.cpp
      ${production}/src/swerve_setpoint.cpp
      ${production}/src/swerve_odometry.cpp
      ${production}/src/odometry_covariance.cpp)
    target_compile_features(nav_compat_prediction PUBLIC cxx_std_17)
    target_include_directories(nav_compat_prediction PUBLIC
      ${prediction}/include ${production}/include)
    target_link_libraries(nav_compat_prediction PUBLIC ruckig::ruckig)
    ament_add_gtest(test_navigation_consumer test/test_navigation_consumer.cpp)
    target_include_directories(test_navigation_consumer PRIVATE ${FOLLOW_MPPI_REFERENCE_DIR})
    target_link_libraries(test_navigation_consumer swerve_controller nav_compat_prediction)
    ament_target_dependencies(test_navigation_consumer tf2 tf2_geometry_msgs)
    ament_add_gtest(test_navigation_replay
      test/navigation_replay.cpp test/navigation_replay_reference.cpp
      ${production}/src/control_core.cpp
      ${production}/src/body_velocity_shaper.cpp
      ${production}/src/swerve_kinematics.cpp
      ${production}/src/swerve_setpoint.cpp
      ${production}/src/swerve_odometry.cpp
      ${production}/src/odometry_covariance.cpp)
    target_include_directories(test_navigation_replay PRIVATE ${production}/include)
    target_link_libraries(test_navigation_replay swerve_math)
  endif()
  ament_add_gtest(test_controller_manager test/test_controller_manager.cpp)
  target_link_libraries(test_controller_manager swerve_controller)
  ament_target_dependencies(test_controller_manager controller_manager)
  target_compile_definitions(test_controller_manager PRIVATE
    SWERVE_MOCK_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/chassis.urdf")
endif()

ament_package()
