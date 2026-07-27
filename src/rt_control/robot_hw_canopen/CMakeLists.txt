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
set(CANOPEN_GENERATED_FILES
  "${CANOPEN_CONFIG_BUILD_DIR}/bus.yml"
  "${CANOPEN_CONFIG_BUILD_DIR}/master.dcf"
  "${CANOPEN_CONFIG_BUILD_DIR}/master.bin"
  "${CANOPEN_CONFIG_BUILD_DIR}/left_track_joint.bin"
  "${CANOPEN_CONFIG_BUILD_DIR}/right_track_joint.bin"
  "${CANOPEN_CONFIG_BUILD_DIR}/eds/ld2_drive.eds"
  "${CANOPEN_CONFIG_BUILD_DIR}/eds-normalization.log"
)
file(MAKE_DIRECTORY "${CANOPEN_CONFIG_BUILD_DIR}")

add_custom_command(
  OUTPUT ${CANOPEN_GENERATED_FILES}
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
  DEPENDS
    "${CANOPEN_CONFIG_SOURCE_DIR}/bus.yml"
    "${CANOPEN_CONFIG_SOURCE_DIR}/eds/ld2_drive.eds"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/normalize_leadshine_eds.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ResolveBusConfigPath.cmake"
  WORKING_DIRECTORY "${CANOPEN_CONFIG_BUILD_DIR}"
  COMMENT "Generating the CANopen runtime DCF from the fixed Leadshine authority EDS"
  VERBATIM
)
add_custom_target(canopen_runtime_config ALL DEPENDS ${CANOPEN_GENERATED_FILES})

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY "${CANOPEN_CONFIG_BUILD_DIR}/" DESTINATION share/${PROJECT_NAME}/config
  PATTERN "bus.preprocessed.yml" EXCLUDE)

ament_package()
