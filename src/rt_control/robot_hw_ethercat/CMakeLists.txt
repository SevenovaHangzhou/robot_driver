cmake_minimum_required(VERSION 3.8)
project(robot_hw_ethercat)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ti5_polarity_profiles test/test_ti5_polarity_profiles.py)
  ament_add_pytest_test(test_x503_profiles test/test_x503_profiles.py)
  ament_add_pytest_test(test_x503_topology test/test_x503_topology.py)
endif()

install(DIRECTORY urdf config DESTINATION share/${PROJECT_NAME})

ament_package()
