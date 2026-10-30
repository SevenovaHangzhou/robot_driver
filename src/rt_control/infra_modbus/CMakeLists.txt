cmake_minimum_required(VERSION 3.8)
project(infra_modbus)
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)

find_package(ament_cmake REQUIRED)

add_library(infra_modbus SHARED src/client.cpp)
target_compile_features(infra_modbus PUBLIC cxx_std_17)
target_include_directories(infra_modbus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS infra_modbus EXPORT export_infra_modbus
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_infra_modbus HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Threads REQUIRED)
  ament_add_gtest(test_client test/test_client.cpp TIMEOUT 60)
  target_link_libraries(test_client infra_modbus Threads::Threads)
endif()

ament_package()
