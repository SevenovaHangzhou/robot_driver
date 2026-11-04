cmake_minimum_required(VERSION 3.8)
project(hw_semantic_components)
# Uniform warnings for all owned targets, including ament test targets.
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(hw_semantic_components INTERFACE)
target_compile_features(hw_semantic_components INTERFACE cxx_std_17)
target_include_directories(hw_semantic_components INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hw_semantic_components INTERFACE
  hardware_interface::hardware_interface
)

install(TARGETS hw_semantic_components
  EXPORT export_rt_control_semantic_components
)
install(DIRECTORY include/ DESTINATION include)
# Python twin of cia402_state.hpp for offline tools (hw_ethercat preflight).
ament_python_install_package(${PROJECT_NAME})

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
    hw_semantic_components
  )
  target_compile_definitions(test_cia402_axis PRIVATE
    CIA402_CASES_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/cia402_cases.txt")

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_cia402_python test/test_cia402_python.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})

  ament_add_gtest(test_force_torque_sensor
    test/test_force_torque_sensor.cpp
  )
  target_compile_features(test_force_torque_sensor PUBLIC cxx_std_17)
  target_compile_options(test_force_torque_sensor PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion
  )
  target_include_directories(test_force_torque_sensor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_force_torque_sensor
    hw_semantic_components
  )
endif()

ament_package()
