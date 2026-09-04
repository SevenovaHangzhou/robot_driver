cmake_minimum_required(VERSION 3.8)
project(robot_hw_canopen)

find_package(ament_cmake REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

find_program(COGEN_EXECUTABLE NAMES cogen REQUIRED)
find_program(DCFGEN_EXECUTABLE NAMES dcfgen REQUIRED)

set(CANOPEN_CONFIG_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/config")
set(CANOPEN_CONFIG_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_canopen_config")
set(CANOPEN_RUNTIME_CONFIG_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config")
set(CANOPEN_VARIANT_VALIDATOR "${CMAKE_CURRENT_SOURCE_DIR}/cmake/validate_canopen_variant.py")
set(CANOPEN_GENERATION_STAMP "${CANOPEN_CONFIG_BUILD_DIR}/generation.stamp")
set(CANOPEN_STATIC_BYPRODUCTS
  "${CANOPEN_CONFIG_BUILD_DIR}/bus.yml"
  "${CANOPEN_CONFIG_BUILD_DIR}/bus.authority.yml"
  "${CANOPEN_CONFIG_BUILD_DIR}/bus.preprocessed.yml"
  "${CANOPEN_CONFIG_BUILD_DIR}/master.dcf"
  "${CANOPEN_CONFIG_BUILD_DIR}/master.bin"
  "${CANOPEN_CONFIG_BUILD_DIR}/eds/ld2_drive.eds"
  "${CANOPEN_CONFIG_BUILD_DIR}/eds-normalization.log"
)

file(GLOB CANOPEN_VARIANT_PATHS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/variants/*.yaml"
)
if(NOT CANOPEN_VARIANT_PATHS)
  message(FATAL_ERROR "At least one CANopen variant descriptor is required")
endif()

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${CANOPEN_VARIANT_PATHS}
  "${CANOPEN_CONFIG_SOURCE_DIR}/bus.yml"
)
set(CANOPEN_NODE_BIN_NAMES)
set(CANOPEN_VARIANT_BUILD_VALIDATION_COMMANDS)
foreach(CANOPEN_VARIANT_PATH IN LISTS CANOPEN_VARIANT_PATHS)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${CANOPEN_VARIANT_VALIDATOR}"
      --variant "${CANOPEN_VARIANT_PATH}"
      --bus "${CANOPEN_CONFIG_SOURCE_DIR}/bus.yml"
      --print-node-bin-names
    RESULT_VARIABLE CANOPEN_VARIANT_VALIDATION_RESULT
    OUTPUT_VARIABLE CANOPEN_NODE_BIN_NAMES_RAW
    ERROR_VARIABLE CANOPEN_VARIANT_VALIDATION_ERROR
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT CANOPEN_VARIANT_VALIDATION_RESULT EQUAL 0)
    message(FATAL_ERROR
      "CANopen variant validation failed during configure for "
      "${CANOPEN_VARIANT_PATH}: ${CANOPEN_VARIANT_VALIDATION_ERROR}"
    )
  endif()
  string(REPLACE "\n" ";" CANOPEN_VARIANT_NODE_BIN_NAMES
    "${CANOPEN_NODE_BIN_NAMES_RAW}"
  )
  list(APPEND CANOPEN_NODE_BIN_NAMES ${CANOPEN_VARIANT_NODE_BIN_NAMES})
  list(APPEND CANOPEN_VARIANT_BUILD_VALIDATION_COMMANDS
    COMMAND "${Python3_EXECUTABLE}" "${CANOPEN_VARIANT_VALIDATOR}"
      --variant "${CANOPEN_VARIANT_PATH}"
      --bus "${CANOPEN_CONFIG_SOURCE_DIR}/bus.yml"
  )
endforeach()
list(REMOVE_DUPLICATES CANOPEN_NODE_BIN_NAMES)
set(CANOPEN_NODE_BIN_BYPRODUCTS)
foreach(CANOPEN_NODE_BIN_NAME IN LISTS CANOPEN_NODE_BIN_NAMES)
  list(APPEND CANOPEN_NODE_BIN_BYPRODUCTS
    "${CANOPEN_CONFIG_BUILD_DIR}/${CANOPEN_NODE_BIN_NAME}"
  )
endforeach()

file(MAKE_DIRECTORY "${CANOPEN_CONFIG_BUILD_DIR}")

add_custom_command(
  OUTPUT "${CANOPEN_GENERATION_STAMP}"
  BYPRODUCTS ${CANOPEN_STATIC_BYPRODUCTS} ${CANOPEN_NODE_BIN_BYPRODUCTS}
  ${CANOPEN_VARIANT_BUILD_VALIDATION_COMMANDS}
  COMMAND "${CMAKE_COMMAND}" -E copy_if_different
    "${CANOPEN_CONFIG_SOURCE_DIR}/bus.yml"
    "${CANOPEN_CONFIG_BUILD_DIR}/bus.authority.yml"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${CANOPEN_CONFIG_BUILD_DIR}/eds"
  COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/normalize_leadshine_eds.py"
    --input "${CANOPEN_CONFIG_SOURCE_DIR}/eds/ld2_drive.eds"
    --output "${CANOPEN_CONFIG_BUILD_DIR}/eds/ld2_drive.eds"
    --log "${CANOPEN_CONFIG_BUILD_DIR}/eds-normalization.log"
  COMMAND "${COGEN_EXECUTABLE}"
    --input-file "${CANOPEN_CONFIG_SOURCE_DIR}/bus.yml"
    --output-file "${CANOPEN_CONFIG_BUILD_DIR}/bus.preprocessed.yml"
  COMMAND "${CMAKE_COMMAND}"
    "-DINPUT_FILE=${CANOPEN_CONFIG_BUILD_DIR}/bus.preprocessed.yml"
    "-DOUTPUT_FILE=${CANOPEN_CONFIG_BUILD_DIR}/bus.yml"
    "-DBUS_CONFIG_PATH=${CANOPEN_RUNTIME_CONFIG_DIR}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ResolveBusConfigPath.cmake"
  COMMAND "${DCFGEN_EXECUTABLE}" -v -d "${CANOPEN_CONFIG_BUILD_DIR}" -rS
    "${CANOPEN_CONFIG_BUILD_DIR}/bus.yml"
  COMMAND "${CMAKE_COMMAND}" -E touch "${CANOPEN_GENERATION_STAMP}"
  DEPENDS
    ${CANOPEN_VARIANT_PATHS}
    "${CANOPEN_VARIANT_VALIDATOR}"
    "${CANOPEN_CONFIG_SOURCE_DIR}/bus.yml"
    "${CANOPEN_CONFIG_SOURCE_DIR}/eds/ld2_drive.eds"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/normalize_leadshine_eds.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ResolveBusConfigPath.cmake"
  WORKING_DIRECTORY "${CANOPEN_CONFIG_BUILD_DIR}"
  COMMENT "Generating the CANopen runtime DCF from the fixed Leadshine authority EDS"
  VERBATIM
)
add_custom_target(canopen_runtime_config ALL DEPENDS "${CANOPEN_GENERATION_STAMP}")

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY variants DESTINATION share/${PROJECT_NAME})
install(DIRECTORY "${CANOPEN_CONFIG_BUILD_DIR}/" DESTINATION share/${PROJECT_NAME}/config
  PATTERN "bus.preprocessed.yml" EXCLUDE
  PATTERN "generation.stamp" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_canopen_hardware_variant
    test/test_canopen_hardware_variant.py
  )
endif()

ament_package()
