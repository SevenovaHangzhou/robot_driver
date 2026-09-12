cmake_minimum_required(VERSION 3.8)
project(robot_hw_ethercat)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(ethercat_generic_slave REQUIRED)
find_package(ethercat_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(zeroerr_pp_slave SHARED src/zeroerr_pp_slave.cpp)
target_compile_features(zeroerr_pp_slave PUBLIC cxx_std_17)
target_compile_options(zeroerr_pp_slave PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_include_directories(zeroerr_pp_slave PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(zeroerr_pp_slave ethercat_generic_slave ethercat_interface pluginlib)
pluginlib_export_plugin_description_file(ethercat_interface ethercat_plugins.xml)

add_custom_target(
  validate_ethercat_variants ALL
  COMMAND
    ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_ethercat_variants.py
    --variants-dir ${CMAKE_CURRENT_SOURCE_DIR}/variants
    --profile-dir ${CMAKE_CURRENT_SOURCE_DIR}/config/slaves
  COMMENT "Validating EtherCAT variant, family, and slave-profile contracts"
  VERBATIM
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(gripper_controllers REQUIRED)
  find_package(ethercat_generic_cia402_drive REQUIRED)
  ament_add_gtest(test_stationary_drive test/test_stationary_drive.cpp)
  target_compile_features(test_stationary_drive PUBLIC cxx_std_17)
  ament_target_dependencies(test_stationary_drive ethercat_generic_cia402_drive)
  ament_add_gtest(test_pp_gripper test/test_pp_gripper.cpp)
  target_include_directories(test_pp_gripper PRIVATE include)
  target_compile_features(test_pp_gripper PRIVATE cxx_std_17)
  ament_add_gtest(test_zeroerr_pp_slave test/test_zeroerr_pp_slave.cpp)
  target_link_libraries(test_zeroerr_pp_slave zeroerr_pp_slave)
  target_compile_definitions(test_zeroerr_pp_slave PRIVATE
    PP_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/pp_gripper.yaml")
  ament_add_gtest(test_pp_controller test/test_pp_controller.cpp)
  target_compile_features(test_pp_controller PRIVATE cxx_std_17)
  target_link_libraries(test_pp_controller zeroerr_pp_slave)
  target_compile_definitions(test_pp_controller PRIVATE
    PP_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/pp_gripper.yaml")
  ament_target_dependencies(test_pp_controller gripper_controllers)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ti5_polarity_profiles test/test_ti5_polarity_profiles.py)
  ament_add_pytest_test(test_x503_profiles test/test_x503_profiles.py)
  ament_add_pytest_test(test_x503_topology test/test_x503_topology.py)
  ament_add_pytest_test(
    test_ethercat_hardware_variant
    test/test_ethercat_hardware_variant.py
  )
endif()

install(DIRECTORY urdf config variants DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(TARGETS zeroerr_pp_slave EXPORT export_zeroerr_pp_slave
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_zeroerr_pp_slave HAS_LIBRARY_TARGET)
ament_export_dependencies(ethercat_generic_slave ethercat_interface pluginlib)
install(
  PROGRAMS scripts/validate_ethercat_variants.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
