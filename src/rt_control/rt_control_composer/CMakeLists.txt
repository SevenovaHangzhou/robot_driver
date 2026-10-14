cmake_minimum_required(VERSION 3.8)
project(rt_control_composer)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# Copy schemas even with --symlink-install: the resolver confines refs to installed files.
install(DIRECTORY schema DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.schema.json" PATTERN "README.md")
install(FILES rt_control_composer/renderer_contract.md
  DESTINATION share/${PROJECT_NAME}/rt_control_composer)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml")
install(PROGRAMS scripts/validate_config_files.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(chassis_composition test/test_chassis.py TIMEOUT 60)
  ament_add_pytest_test(composition test/test_compose.py TIMEOUT 60)
  ament_add_pytest_test(config_contract test/test_config_contract.py
    ENV COMPOSER_SCHEMA_DIR=${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/schema)
endif()

ament_package()
