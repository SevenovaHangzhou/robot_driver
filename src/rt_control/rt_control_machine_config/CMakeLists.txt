cmake_minimum_required(VERSION 3.8)
project(rt_control_machine_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY devices machines units evidence
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
