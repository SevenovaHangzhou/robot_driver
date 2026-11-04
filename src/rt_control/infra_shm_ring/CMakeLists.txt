cmake_minimum_required(VERSION 3.8)
project(infra_shm_ring)
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)

find_package(ament_cmake REQUIRED)

add_library(infra_shm_ring INTERFACE)
target_compile_features(infra_shm_ring INTERFACE cxx_std_17)
target_include_directories(infra_shm_ring INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS infra_shm_ring EXPORT export_infra_shm_ring)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_infra_shm_ring HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The writer must produce the frozen v1 sample shared with the Runtime-LOG reader.
  ament_add_gtest(test_shm_ring test/test_shm_ring.cpp)
  target_compile_definitions(test_shm_ring PRIVATE
    RUNTIME_LOG_GOLDEN="${CMAKE_CURRENT_SOURCE_DIR}/test/data/shm_format_v1.bin"
  )
  target_link_libraries(test_shm_ring infra_shm_ring)
endif()

ament_package()
