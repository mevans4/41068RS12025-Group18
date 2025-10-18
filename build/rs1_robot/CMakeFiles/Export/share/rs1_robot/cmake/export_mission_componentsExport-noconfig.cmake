#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rs1_robot::mission_planner_component" for configuration ""
set_property(TARGET rs1_robot::mission_planner_component APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rs1_robot::mission_planner_component PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmission_planner_component.so"
  IMPORTED_SONAME_NOCONFIG "libmission_planner_component.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rs1_robot::mission_planner_component )
list(APPEND _IMPORT_CHECK_FILES_FOR_rs1_robot::mission_planner_component "${_IMPORT_PREFIX}/lib/libmission_planner_component.so" )

# Import target "rs1_robot::drone_controller_enhanced_component" for configuration ""
set_property(TARGET rs1_robot::drone_controller_enhanced_component APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rs1_robot::drone_controller_enhanced_component PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdrone_controller_enhanced_component.so"
  IMPORTED_SONAME_NOCONFIG "libdrone_controller_enhanced_component.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rs1_robot::drone_controller_enhanced_component )
list(APPEND _IMPORT_CHECK_FILES_FOR_rs1_robot::drone_controller_enhanced_component "${_IMPORT_PREFIX}/lib/libdrone_controller_enhanced_component.so" )

# Import target "rs1_robot::pid_controller" for configuration ""
set_property(TARGET rs1_robot::pid_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rs1_robot::pid_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libpid_controller.so"
  IMPORTED_SONAME_NOCONFIG "libpid_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rs1_robot::pid_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_rs1_robot::pid_controller "${_IMPORT_PREFIX}/lib/libpid_controller.so" )

# Import target "rs1_robot::drone_control" for configuration ""
set_property(TARGET rs1_robot::drone_control APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rs1_robot::drone_control PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdrone_control.so"
  IMPORTED_SONAME_NOCONFIG "libdrone_control.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rs1_robot::drone_control )
list(APPEND _IMPORT_CHECK_FILES_FOR_rs1_robot::drone_control "${_IMPORT_PREFIX}/lib/libdrone_control.so" )

# Import target "rs1_robot::state_machine" for configuration ""
set_property(TARGET rs1_robot::state_machine APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rs1_robot::state_machine PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libstate_machine.so"
  IMPORTED_SONAME_NOCONFIG "libstate_machine.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rs1_robot::state_machine )
list(APPEND _IMPORT_CHECK_FILES_FOR_rs1_robot::state_machine "${_IMPORT_PREFIX}/lib/libstate_machine.so" )

# Import target "rs1_robot::path_planner" for configuration ""
set_property(TARGET rs1_robot::path_planner APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rs1_robot::path_planner PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libpath_planner.so"
  IMPORTED_SONAME_NOCONFIG "libpath_planner.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rs1_robot::path_planner )
list(APPEND _IMPORT_CHECK_FILES_FOR_rs1_robot::path_planner "${_IMPORT_PREFIX}/lib/libpath_planner.so" )

# Import target "rs1_robot::mission_executor" for configuration ""
set_property(TARGET rs1_robot::mission_executor APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rs1_robot::mission_executor PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmission_executor.so"
  IMPORTED_SONAME_NOCONFIG "libmission_executor.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rs1_robot::mission_executor )
list(APPEND _IMPORT_CHECK_FILES_FOR_rs1_robot::mission_executor "${_IMPORT_PREFIX}/lib/libmission_executor.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
