cmake_minimum_required(VERSION 3.8)
project(bms_node)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/bms_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bms_protocol test/test_bms_protocol.py)
  ament_add_pytest_test(test_bms_node test/test_bms_node.py)
endif()

ament_package()
