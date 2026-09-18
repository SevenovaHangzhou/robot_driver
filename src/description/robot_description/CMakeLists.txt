cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_v3_description_semantics
    test/test_v3_description_semantics.py
  )
  ament_add_pytest_test(
    test_v3_chassis_semantics
    test/test_v3_chassis_semantics.py
  )
  ament_add_pytest_test(
    test_v3_head_semantics
    test/test_v3_head_semantics.py
  )
endif()

install(DIRECTORY urdf srdf meshes config DESTINATION share/${PROJECT_NAME})

ament_package()
