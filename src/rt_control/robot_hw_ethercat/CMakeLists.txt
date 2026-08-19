cmake_minimum_required(VERSION 3.8)
project(robot_hw_ethercat)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

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
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ti5_polarity_profiles test/test_ti5_polarity_profiles.py)
  ament_add_pytest_test(
    test_ethercat_hardware_variant
    test/test_ethercat_hardware_variant.py
  )
endif()

install(DIRECTORY urdf config variants DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS scripts/validate_ethercat_variants.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
