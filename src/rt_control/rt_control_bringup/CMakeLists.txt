cmake_minimum_required(VERSION 3.8)
project(rt_control_bringup)
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)

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
    launch/rt_control_force_sensors.launch.py
    launch/rt_control_module.launch.py
    launch/rt_control_enable_only.launch.py
    launch/rt_control_arm_runtime.launch.py
    launch/rt_control_head_runtime.launch.py
    launch/rt_control_interface_runtime.launch.py
    launch/rt_control_ultrasonic.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY
    config/machines
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(
  FILES
    config/alfa_v3_grippers.draft.yaml
    config/alfa_v3_rolling_envelope.mock.yaml
    config/bluepoint_force_sensors.draft.yaml
    config/rt_io.yaml
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  PROGRAMS
    scripts/rt_control_start
    scripts/validate_machine_profile.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(swerve_driver REQUIRED)
  find_package(robot_hw_ethercat REQUIRED)
  ament_add_gtest(test_cyclic_controller test/test_cyclic_controller.cpp TIMEOUT 90)
  target_compile_features(test_cyclic_controller PUBLIC cxx_std_17)
  target_include_directories(test_cyclic_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../swerve_driver/test)
  ament_target_dependencies(test_cyclic_controller swerve_driver robot_hw_ethercat)
  target_compile_definitions(test_cyclic_controller PRIVATE
    KINCO_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/../robot_hw_ethercat/test/fixtures/kinco_cyclic_mode_mock.yaml")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_force_sensor_runtime test/test_force_sensor_runtime.py)
  ament_add_pytest_test(
    test_machine_profile
    test/test_machine_profile.py
  )
  ament_add_pytest_test(test_stationary_enable test/test_stationary_enable.py)
  ament_add_pytest_test(test_ultrasonic_launch test/test_ultrasonic_launch.py)
  ament_add_pytest_test(
    test_interface_runtime_launch
    test/test_interface_runtime_launch.py
  )
  ament_add_pytest_test(
    test_arm_motion_runtime
    test/test_arm_motion_runtime.py
  )
  ament_add_pytest_test(test_jtc_only_launch test/test_jtc_only_launch.py)
  ament_add_pytest_test(test_v3_head_can_integration test/test_v3_head_can_integration.py)
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
