cmake_minimum_required(VERSION 3.16)
project(swerve_driver LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
set(dependencies controller_interface hardware_interface diagnostic_msgs geometry_msgs
  nav_msgs sensor_msgs pluginlib rclcpp realtime_tools robot_interfaces_qos rt_control_semantic_components)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

option(SWERVE_ENABLE_COVERAGE "Instrument swerve targets for GCC coverage" OFF)
add_library(swerve_math SHARED
  src/swerve_kinematics.cpp src/swerve_odometry.cpp src/swerve_setpoint.cpp
  src/odometry_covariance.cpp src/control_core.cpp)
add_library(swerve_controller SHARED src/swerve_controller.cpp)
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
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)
  foreach(test kinematics odometry odometry_covariance control_core)
    ament_add_gtest(test_${test} test/test_${test}.cpp)
    target_link_libraries(test_${test} swerve_math)
  endforeach()
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller swerve_controller)
  ament_add_gtest(test_controller_manager test/test_controller_manager.cpp)
  target_link_libraries(test_controller_manager swerve_controller)
  ament_target_dependencies(test_controller_manager controller_manager)
  target_compile_definitions(test_controller_manager PRIVATE
    SWERVE_MOCK_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/chassis.urdf")
endif()

ament_package()
