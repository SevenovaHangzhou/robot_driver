cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_current_model test/test_current_model.py)
endif()

# Mesh assets are classified by robot part; URDF package:// paths match this layout.
install(DIRECTORY urdf srdf config mesh DESTINATION share/${PROJECT_NAME})

ament_package()
