cmake_minimum_required(VERSION 3.8)
project(x503_force_sensor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_path(
  ETHERCAT_MANAGER_INCLUDE_DIR
  NAMES ethercat_manager/ec_master_async.hpp
  PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/ecat_icube/ethercat_manager/include
    ENV X503_ETHERCAT_MANAGER_INCLUDE_DIR
  REQUIRED
)
find_path(
  ETHERLAB_INCLUDE_DIR
  NAMES ecrt.h
  PATHS
    ENV RT_CONTROL_ETHERLAB_PREFIX
    /usr/local/etherlab
  PATH_SUFFIXES include
  REQUIRED
)

add_executable(x503_sdo_snapshot src/x503_sdo_snapshot_node.cpp)
target_compile_features(x503_sdo_snapshot PUBLIC cxx_std_17)
target_compile_options(x503_sdo_snapshot PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_include_directories(
  x503_sdo_snapshot PRIVATE
  "${ETHERCAT_MANAGER_INCLUDE_DIR}"
  "${ETHERLAB_INCLUDE_DIR}"
)
ament_target_dependencies(x503_sdo_snapshot diagnostic_msgs rclcpp yaml_cpp_vendor)
install(TARGETS x503_sdo_snapshot DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/x503_wrench_bridge DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bridge test/test_bridge.py)
  ament_add_pytest_test(test_sdo_contract test/test_sdo_contract.py)
endif()

ament_package()
