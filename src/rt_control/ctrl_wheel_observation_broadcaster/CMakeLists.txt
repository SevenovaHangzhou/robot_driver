cmake_minimum_required(VERSION 3.16)
project(ctrl_wheel_observation_broadcaster LANGUAGES CXX)
# Uniform warnings for all owned targets, including ament test targets.
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)

find_package(ament_cmake REQUIRED)
set(dependencies
  controller_interface
  ctrl_swerve
  hardware_interface
  hw_semantic_components
  pluginlib
  rclcpp
  realtime_tools
  robot_interfaces_qos
  robot_rt_control_interfaces
)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(wheel_observation_broadcaster SHARED
  src/channel_tracker.cpp
  src/wheel_observation_broadcaster.cpp
)
target_compile_features(wheel_observation_broadcaster PUBLIC cxx_std_17)
target_include_directories(wheel_observation_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(wheel_observation_broadcaster PUBLIC ${dependencies})

pluginlib_export_plugin_description_file(
  controller_interface wheel_observation_broadcaster_plugins.xml
)

install(
  TARGETS wheel_observation_broadcaster
  EXPORT export_wheel_observation_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_wheel_observation_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_channel_tracker test/test_channel_tracker.cpp)
  target_link_libraries(test_channel_tracker wheel_observation_broadcaster)
  ament_add_gtest(test_wheel_observation_broadcaster test/test_wheel_observation_broadcaster.cpp)
  target_link_libraries(test_wheel_observation_broadcaster wheel_observation_broadcaster)
endif()

ament_package()
