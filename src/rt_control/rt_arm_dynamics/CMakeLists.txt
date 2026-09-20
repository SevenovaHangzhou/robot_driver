cmake_minimum_required(VERSION 3.16)
project(rt_arm_dynamics LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

add_library(rt_arm_dynamics SHARED src/model.cpp src/arm_dynamics.cpp)
target_compile_features(rt_arm_dynamics PUBLIC cxx_std_17)
target_compile_options(rt_arm_dynamics PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_include_directories(rt_arm_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(rt_arm_dynamics PUBLIC pinocchio::pinocchio PRIVATE tinyxml2::tinyxml2)
install(TARGETS rt_arm_dynamics EXPORT export_rt_arm_dynamics
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_rt_arm_dynamics HAS_LIBRARY_TARGET)
ament_export_dependencies(pinocchio)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Instrument our algorithm translation units too, not just the test caller.
  target_compile_definitions(rt_arm_dynamics PUBLIC EIGEN_RUNTIME_NO_MALLOC)
  ament_add_gtest(test_arm_dynamics test/test_arm_dynamics.cpp test/heap_probe.cpp)
  target_link_libraries(test_arm_dynamics rt_arm_dynamics)
  target_compile_definitions(test_arm_dynamics PRIVATE
    ARM_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/pendulum.urdf"
    ARM_SEVEN_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/dual_seven.urdf")
endif()
ament_package()
