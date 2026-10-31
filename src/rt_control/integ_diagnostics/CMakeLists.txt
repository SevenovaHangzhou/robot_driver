cmake_minimum_required(VERSION 3.8)
project(integ_diagnostics)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_report test/test_report.py)
endif()

ament_package()
