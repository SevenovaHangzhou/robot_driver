cmake_minimum_required(VERSION 3.8)
project(plc_node)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/plc_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_modbus_tcp test/test_modbus_tcp.py)
  ament_add_pytest_test(test_plc_interface test/test_plc_interface.py)
  ament_add_pytest_test(test_plc_logic test/test_plc_logic.py)
  ament_add_pytest_test(test_plc_node test/test_plc_node.py)
  ament_add_pytest_test(test_plc_runtime test/test_plc_runtime.py)
endif()

ament_package()
