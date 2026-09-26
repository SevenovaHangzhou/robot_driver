cmake_minimum_required(VERSION 3.8)
project(rt_control_operator_web)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/rt_control_operator_web
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
)

# test/mock/** is deliberately not installed: the Mock chassis loop is a
# test-only asset and must never become a production launch entry.

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_teleop test/test_teleop.py)
  ament_add_pytest_test(test_config test/test_config.py)
  ament_add_pytest_test(test_protocol test/test_protocol.py)
  ament_add_pytest_test(test_server test/test_server.py)
  ament_add_pytest_test(test_robot_model test/test_robot_model.py)
  ament_add_pytest_test(test_mock_loop test/test_mock_loop.py
    TIMEOUT 180
    ENV ROS_DOMAIN_ID=109 ROS_LOCALHOST_ONLY=1)
endif()

ament_package()
