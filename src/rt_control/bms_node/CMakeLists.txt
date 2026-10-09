cmake_minimum_required(VERSION 3.8)
project(bms_node)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)
find_package(robot_interfaces_qos REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(bms_protocol src/protocol.cpp)
target_compile_features(bms_protocol PUBLIC cxx_std_17)
target_include_directories(bms_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(bms_dual_battery_controller src/dual_battery_controller.cpp)
target_compile_features(bms_dual_battery_controller PUBLIC cxx_std_17)
target_include_directories(bms_dual_battery_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(bms_dual_battery_controller PUBLIC bms_protocol)

add_library(bms_can_transport src/can_transport.cpp)
target_compile_features(bms_can_transport PUBLIC cxx_std_17)
target_include_directories(bms_can_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(bms_node_component SHARED src/bms_node.cpp)
target_compile_features(bms_node_component PUBLIC cxx_std_17)
target_include_directories(bms_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(bms_node_component
  bms_protocol bms_dual_battery_controller bms_can_transport Threads::Threads)
ament_target_dependencies(
  bms_node_component
  rclcpp
  robot_interfaces_qos
  sensor_msgs
  std_msgs
)

add_executable(bms_node src/main.cpp)
target_compile_features(bms_node PRIVATE cxx_std_17)
target_link_libraries(bms_node bms_node_component)
ament_target_dependencies(bms_node rclcpp)

install(
  TARGETS bms_protocol bms_dual_battery_controller bms_can_transport bms_node_component
  EXPORT export_bms_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS bms_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_bms_node HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp robot_interfaces_qos sensor_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_bms_protocol test/test_bms_protocol.cpp)
  target_compile_features(test_bms_protocol PRIVATE cxx_std_17)
  target_link_libraries(test_bms_protocol bms_protocol)

  ament_add_gtest(test_dual_battery_controller test/test_dual_battery_controller.cpp)
  target_compile_features(test_dual_battery_controller PRIVATE cxx_std_17)
  target_link_libraries(test_dual_battery_controller bms_dual_battery_controller)

  ament_add_gtest(test_can_transport test/test_can_transport.cpp)
  target_compile_features(test_can_transport PRIVATE cxx_std_17)
  target_link_libraries(test_can_transport bms_can_transport)

  ament_add_gtest(test_bms_control_node test/test_bms_control_node.cpp)
  target_compile_features(test_bms_control_node PRIVATE cxx_std_17)
  target_link_libraries(test_bms_control_node bms_node_component)
  ament_target_dependencies(test_bms_control_node rclcpp std_msgs)

  ament_add_gtest(test_bms_node test/test_bms_node.cpp)
  target_compile_features(test_bms_node PRIVATE cxx_std_17)
  target_link_libraries(test_bms_node bms_node_component)
  ament_target_dependencies(
    test_bms_node
    rclcpp
    robot_interfaces_qos
    sensor_msgs
  )
endif()

ament_package()
