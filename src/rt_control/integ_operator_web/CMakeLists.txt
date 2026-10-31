cmake_minimum_required(VERSION 3.8)
project(integ_operator_web)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/integ_operator_web DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY web config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(test protocol teleop config server tuning launcher)
    ament_add_pytest_test(test_${test} test/test_${test}.py TIMEOUT 60)
  endforeach()
endif()

ament_package()
