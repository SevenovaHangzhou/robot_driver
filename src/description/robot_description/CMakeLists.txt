cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_current_model test/test_current_model.py)
  ament_add_pytest_test(test_motion_scope test/test_motion_scope.py)
endif()

# Mesh assets are classified by robot part; URDF package:// paths match this layout.
install(DIRECTORY urdf srdf config mesh DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tools DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE)

ament_package()
