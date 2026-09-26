cmake_minimum_required(VERSION 3.8)
project(robot_hw_canopen)

find_package(ament_cmake REQUIRED)
find_package(canopen_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclcpp REQUIRED)

add_library(swerve_encoder_feedback SHARED src/swerve_encoder_feedback.cpp)
target_compile_features(swerve_encoder_feedback PUBLIC cxx_std_17)
target_compile_options(
  swerve_encoder_feedback PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_include_directories(swerve_encoder_feedback PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(swerve_encoder_system SHARED src/swerve_encoder_system.cpp)
target_compile_features(swerve_encoder_system PUBLIC cxx_std_17)
target_compile_options(
  swerve_encoder_system PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_include_directories(swerve_encoder_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(swerve_encoder_system swerve_encoder_feedback)
ament_target_dependencies(
  swerve_encoder_system canopen_ros2_control hardware_interface pluginlib rclcpp)
pluginlib_export_plugin_description_file(
  hardware_interface robot_hw_canopen_plugins.xml)

set(SWERVE_ENCODER_PROFILE_VALIDATOR
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/validate_swerve_encoder_profile.py")
set(SWERVE_ENCODER_DRAFT_PROFILE
  "${CMAKE_CURRENT_SOURCE_DIR}/config/machines/alfa_v3_swerve_encoders.draft.yaml")
add_custom_target(
  validate_swerve_encoder_profile ALL
  COMMAND "${Python3_EXECUTABLE}" "${SWERVE_ENCODER_PROFILE_VALIDATOR}"
    --profile "${SWERVE_ENCODER_DRAFT_PROFILE}"
  COMMENT "Validating fail-closed alfa_v3 swerve encoder draft"
  VERBATIM)

install(TARGETS swerve_encoder_feedback swerve_encoder_system
  EXPORT export_swerve_encoder_feedback
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES robot_hw_canopen_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/machines DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS cmake/validate_swerve_encoder_profile.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_swerve_encoder_feedback test/test_swerve_encoder_feedback.cpp)
  target_link_libraries(test_swerve_encoder_feedback swerve_encoder_feedback)
  ament_add_gtest(test_swerve_encoder_system test/test_swerve_encoder_system.cpp)
  target_link_libraries(test_swerve_encoder_system swerve_encoder_system)
  ament_target_dependencies(
    test_swerve_encoder_system canopen_ros2_control hardware_interface rclcpp)
endif()

ament_export_targets(export_swerve_encoder_feedback HAS_LIBRARY_TARGET)
ament_export_dependencies(
  canopen_ros2_control hardware_interface pluginlib rclcpp)
ament_package()
