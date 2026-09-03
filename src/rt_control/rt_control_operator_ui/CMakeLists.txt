cmake_minimum_required(VERSION 3.8)
project(rt_control_operator_ui)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/rt_control_operator_ui DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_app_lifecycle test/test_app_lifecycle.py)
  ament_add_pytest_test(test_command_gateway test/test_command_gateway.py)
  ament_add_pytest_test(test_commands test/test_commands.py)
  ament_add_pytest_test(test_diagnostic_adapter test/test_diagnostic_adapter.py)
  ament_add_pytest_test(test_fault_catalog test/test_fault_catalog.py)
  ament_add_pytest_test(test_fault_log test/test_fault_log.py)
  ament_add_pytest_test(test_main_window test/test_main_window.py)
  ament_add_pytest_test(test_presentation test/test_presentation.py)
  ament_add_pytest_test(test_presenter test/test_presenter.py)
endif()

ament_package()
