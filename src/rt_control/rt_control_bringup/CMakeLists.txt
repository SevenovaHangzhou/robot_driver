cmake_minimum_required(VERSION 3.8)
project(rt_control_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

ament_python_install_package(${PROJECT_NAME})

add_custom_target(
  validate_machine_profile ALL
  COMMAND
    ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_machine_profile.py
    --manifest ${CMAKE_CURRENT_SOURCE_DIR}/config/machines/alfa_v3.yaml
    --all
  COMMENT "Validating ELECTRI-118 machine profiles and control scopes"
  VERBATIM
)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(
  PROGRAMS
    scripts/rt_control_start
    scripts/validate_machine_profile.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(
    test_hardware_composition_contract
    test/test_hardware_composition_contract.py
  )
  ament_add_pytest_test(
    test_hardware_composition_config
    test/test_hardware_composition_config.py
  )
  ament_add_pytest_test(test_preop_snapshot_launch test/test_preop_snapshot_launch.py)
  ament_add_pytest_test(
    test_machine_profile
    test/test_machine_profile.py
  )
  ament_add_pytest_test(test_stationary_enable test/test_stationary_enable.py)
  ament_add_pytest_test(
    test_machine_profile_launch
    test/test_machine_profile_launch.py
  )
  ament_add_pytest_test(
    test_machine_profile_validator
    test/test_machine_profile_validator.py
  )
  # ELECTRI-77: mock cross-domain contract suite (TC-RO-01 / TC-RO-02).
  # Requires an isolated ROS_DOMAIN_ID when run on a machine with a live
  # ROS 2 graph (the repo tooling exports ROS_DOMAIN_ID=142 for this).
  add_launch_test(
    test/test_mock_contract.py
    TIMEOUT 300
  )
endif()

ament_package()
