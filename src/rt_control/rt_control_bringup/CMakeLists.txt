cmake_minimum_required(VERSION 3.8)
project(rt_control_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(PROGRAMS scripts/rt_control_start DESTINATION lib/${PROJECT_NAME})

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
  # ELECTRI-77: mock cross-domain contract suite (TC-RO-01 / TC-RO-02).
  # Requires an isolated ROS_DOMAIN_ID when run on a machine with a live
  # ROS 2 graph (the repo tooling exports ROS_DOMAIN_ID=142 for this).
  add_launch_test(
    test/test_mock_contract.py
    TIMEOUT 300
  )
endif()

ament_package()
