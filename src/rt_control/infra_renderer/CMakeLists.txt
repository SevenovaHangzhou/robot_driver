cmake_minimum_required(VERSION 3.8)
project(infra_renderer)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_infra_renderer test/test_infra_renderer.py)
endif()

ament_package()
