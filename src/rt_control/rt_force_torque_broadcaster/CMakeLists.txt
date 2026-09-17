cmake_minimum_required(VERSION 3.16)
project(rt_force_torque_broadcaster LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
set(dependencies
  controller_interface
  diagnostic_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  realtime_tools
  robot_interfaces_qos
  rt_control_semantic_components
  std_msgs
)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(rt_force_torque_broadcaster SHARED
  src/force_torque_broadcaster.cpp
)
target_compile_features(rt_force_torque_broadcaster PUBLIC cxx_std_17)
target_compile_options(rt_force_torque_broadcaster PRIVATE
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow
)
target_include_directories(rt_force_torque_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rt_force_torque_broadcaster PUBLIC ${dependencies})

pluginlib_export_plugin_description_file(
  controller_interface force_torque_broadcaster_plugins.xml
)

install(
  TARGETS rt_force_torque_broadcaster
  EXPORT export_rt_force_torque_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES force_torque_broadcaster_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_rt_force_torque_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_force_torque_processor
    test/test_force_torque_processor.cpp
  )
  target_compile_features(test_force_torque_processor PUBLIC cxx_std_17)
  target_compile_options(test_force_torque_processor PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow
  )
  target_include_directories(test_force_torque_processor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_force_torque_processor
    rt_control_semantic_components
  )

  ament_add_gtest(test_force_torque_broadcaster
    test/test_force_torque_broadcaster.cpp
  )
  target_compile_features(test_force_torque_broadcaster PUBLIC cxx_std_17)
  target_compile_options(test_force_torque_broadcaster PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow
  )
  target_include_directories(test_force_torque_broadcaster PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_force_torque_broadcaster
    rt_force_torque_broadcaster
  )
  ament_target_dependencies(test_force_torque_broadcaster ${dependencies})
endif()

ament_package()
