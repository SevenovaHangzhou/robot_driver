cmake_minimum_required(VERSION 3.8)
project(rt_control_arm_console)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/rt_control_arm_console DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY web config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(trajectory test/test_trajectory.py TIMEOUT 60)
  ament_add_pytest_test(console test/test_console.py TIMEOUT 60)
  ament_add_pytest_test(server test/test_server.py TIMEOUT 60)
endif()

ament_package()
