cmake_minimum_required(VERSION 3.8)
project(integ_operator_web)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces_rolling_client REQUIRED)
pybind11_add_module(_rolling_sender src/rolling_python.cpp)
target_compile_features(_rolling_sender PRIVATE cxx_std_17)
ament_target_dependencies(_rolling_sender PUBLIC rclcpp robot_interfaces_rolling_client)
ament_get_python_install_dir(python_install_dir)
install(TARGETS _rolling_sender LIBRARY DESTINATION ${python_install_dir}/${PROJECT_NAME})

install(PROGRAMS scripts/integ_operator_web DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY web config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(test protocol teleop config server tuning launcher
          arm_trajectory arm_console arm_vacuum_control arm_page arm_authority arm_idle arm_ros_send arm_modes mode_service rolling_output rolling_hold rolling_rejection jog_page head_console head_page status_page)
    ament_add_pytest_test(test_${test} test/test_${test}.py TIMEOUT 60
      APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
  endforeach()
  ament_add_pytest_test(test_rolling_binding test/test_rolling_binding.py TIMEOUT 60
    ENV "JOG_SENDER_EXTENSION=$<TARGET_FILE:_rolling_sender>"
      "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=219")
endif()

ament_package()
