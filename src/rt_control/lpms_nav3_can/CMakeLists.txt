cmake_minimum_required(VERSION 3.13)
project(lpms_nav3_can)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ENABLE_COVERAGE "Enable GCC/Clang coverage instrumentation" OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_interfaces_qos REQUIRED)

add_library(lpms_nav3_decoder src/decoder.cpp)
target_compile_features(lpms_nav3_decoder PUBLIC cxx_std_17)
target_include_directories(lpms_nav3_decoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(lpms_nav3_ros_conversion src/ros_conversion.cpp)
target_compile_features(lpms_nav3_ros_conversion PUBLIC cxx_std_17)
target_include_directories(lpms_nav3_ros_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lpms_nav3_ros_conversion PUBLIC lpms_nav3_decoder)
ament_target_dependencies(lpms_nav3_ros_conversion PUBLIC sensor_msgs)

add_library(lpms_nav3_socket_can src/socket_can.cpp)
target_compile_features(lpms_nav3_socket_can PUBLIC cxx_std_17)
target_include_directories(lpms_nav3_socket_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(lpms_nav3_node SHARED src/lpms_nav3_can_node.cpp)
target_compile_features(lpms_nav3_node PUBLIC cxx_std_17)
target_link_libraries(
  lpms_nav3_node
  lpms_nav3_decoder
  lpms_nav3_ros_conversion
  lpms_nav3_socket_can
)
ament_target_dependencies(
  lpms_nav3_node
  diagnostic_msgs
  rclcpp
  robot_interfaces_qos
  sensor_msgs
)
add_executable(lpms_nav3_can_node src/main.cpp)
target_link_libraries(lpms_nav3_can_node lpms_nav3_node)
ament_target_dependencies(lpms_nav3_can_node rclcpp)

install(
  TARGETS lpms_nav3_decoder lpms_nav3_ros_conversion lpms_nav3_socket_can lpms_nav3_node
  EXPORT export_lpms_nav3_decoder
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS lpms_nav3_can_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md NOTICE.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_lpms_nav3_decoder HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(diagnostic_msgs rclcpp robot_interfaces_qos sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_node_without_can test/test_node_without_can.py TIMEOUT 60)
  ament_add_gtest(test_node test/test_node.cpp)
  target_link_libraries(test_node lpms_nav3_node)
  ament_target_dependencies(test_node diagnostic_msgs rclcpp robot_interfaces_qos sensor_msgs)

  ament_add_gtest(test_decoder test/test_decoder.cpp)
  target_compile_features(test_decoder PRIVATE cxx_std_17)
  target_link_libraries(test_decoder lpms_nav3_decoder)

  ament_add_gtest(test_ros_conversion test/test_ros_conversion.cpp)
  target_compile_features(test_ros_conversion PRIVATE cxx_std_17)
  target_link_libraries(test_ros_conversion lpms_nav3_ros_conversion)
  ament_target_dependencies(test_ros_conversion sensor_msgs)

  ament_add_gtest(test_socket_can_protocol test/test_socket_can_protocol.cpp)
  target_compile_features(test_socket_can_protocol PRIVATE cxx_std_17)
  target_link_libraries(test_socket_can_protocol lpms_nav3_socket_can)
endif()

if(ENABLE_COVERAGE)
  set(coverage_targets
    lpms_nav3_decoder
    lpms_nav3_ros_conversion
    lpms_nav3_socket_can
    lpms_nav3_can_node
    lpms_nav3_node
  )
  if(BUILD_TESTING)
    list(APPEND coverage_targets
      test_decoder
      test_ros_conversion
      test_socket_can_protocol
      test_node
    )
  endif()
  foreach(target IN LISTS coverage_targets)
    target_compile_options(${target} PRIVATE --coverage -O0 -g)
    target_link_options(${target} PRIVATE --coverage)
  endforeach()
endif()

ament_package()
