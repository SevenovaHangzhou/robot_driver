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
  FILES
    launch/rt_control_module.launch.py
    launch/rt_control_enable_only.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY
    config/machines
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  FILES
    config/alfa_v3_grippers.draft.yaml
    config/bluepoint_force_sensors.draft.yaml
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  PROGRAMS
    scripts/rt_control_start
    scripts/validate_machine_profile.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
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
endif()

ament_package()
