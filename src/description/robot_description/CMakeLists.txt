cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_v3_2_2_production_profile
    test/test_v3_2_2_production_profile.py
  )
  ament_add_pytest_test(
    test_v3_2_2_source_profile
    test/test_v3_2_2_source_profile.py
  )
  ament_add_pytest_test(
    test_v3_analytic_proxy_profile
    test/test_v3_analytic_proxy_profile.py
  )
  ament_add_pytest_test(
    test_analytic_proxy_mobile
    test/test_analytic_proxy_mobile.py
  )
endif()

install(DIRECTORY urdf srdf meshes config DESTINATION share/${PROJECT_NAME})
install(
  DIRECTORY model_sources/robot_v3_2_2 model_sources/mechanical_958c49d
  DESTINATION share/${PROJECT_NAME}/model_sources
)

ament_package()
