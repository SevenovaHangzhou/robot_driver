cmake_minimum_required(VERSION 3.16)
project(ctrl_gravity_feedforward LANGUAGES CXX)
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
find_package(ament_cmake REQUIRED)
set(dependencies controller_interface hardware_interface kdl_parser orocos_kdl_vendor
  pluginlib rclcpp hw_semantic_components urdf)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(orocos_kdl REQUIRED)

add_library(${PROJECT_NAME} SHARED src/gravity_model.cpp src/gravity_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${dependencies} orocos_kdl)
pluginlib_export_plugin_description_file(controller_interface plugins.xml)

add_executable(gravity_preview src/gravity_preview.cpp)
target_link_libraries(gravity_preview ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} gravity_preview EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} orocos_kdl)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gravity test/test_gravity.cpp)
  target_link_libraries(test_gravity ${PROJECT_NAME})
endif()
ament_package()
