cmake_minimum_required(VERSION 3.8)
project(x503_force_sensor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/x503_wrench_bridge scripts/x503_sdo_snapshot DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bridge test/test_bridge.py)
  ament_add_pytest_test(test_sdo_contract test/test_sdo_contract.py)
  ament_add_pytest_test(test_preop_snapshot test/test_preop_snapshot.py)
  ament_add_pytest_test(test_snapshot_state test/test_snapshot_state.py)
  ament_add_pytest_test(test_node_snapshot test/test_node_snapshot.py)
endif()

ament_package()
