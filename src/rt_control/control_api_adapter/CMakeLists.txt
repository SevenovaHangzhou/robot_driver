cmake_minimum_required(VERSION 3.8)
project(control_api_adapter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/control_enable_adapter DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_control_enable_adapter test/test_control_enable_adapter.py)
endif()

ament_package()
