cmake_minimum_required(VERSION 3.8)
project(rt_control_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(PROGRAMS scripts/rt_control_start DESTINATION lib/${PROJECT_NAME})

ament_package()
