cmake_minimum_required(VERSION 3.8)
project(rt_control_semantic_components)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(rt_control_semantic_components INTERFACE)
target_compile_features(rt_control_semantic_components INTERFACE cxx_std_17)
target_include_directories(rt_control_semantic_components INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rt_control_semantic_components INTERFACE
  hardware_interface::hardware_interface
)

install(TARGETS rt_control_semantic_components
  EXPORT export_rt_control_semantic_components
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_rt_control_semantic_components HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_cia402_axis
    test/test_cia402_axis.cpp
  )
  target_compile_features(test_cia402_axis PUBLIC cxx_std_17)
  target_compile_options(test_cia402_axis PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
  target_include_directories(test_cia402_axis PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_cia402_axis
    rt_control_semantic_components
  )
endif()

ament_package()
