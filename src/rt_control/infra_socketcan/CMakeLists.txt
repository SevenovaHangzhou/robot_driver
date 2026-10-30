cmake_minimum_required(VERSION 3.8)
project(infra_socketcan)
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)

find_package(ament_cmake REQUIRED)

add_library(infra_socketcan SHARED src/socket.cpp)
target_compile_features(infra_socketcan PUBLIC cxx_std_17)
target_include_directories(infra_socketcan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS infra_socketcan EXPORT export_infra_socketcan
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_infra_socketcan HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_socket test/test_socket.cpp)
  target_link_libraries(test_socket infra_socketcan)
endif()

ament_package()
