#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "drone_ignition_bringup::mission_planner_component" for configuration "Debug"
set_property(TARGET drone_ignition_bringup::mission_planner_component APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(drone_ignition_bringup::mission_planner_component PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmission_planner_component.so"
  IMPORTED_SONAME_DEBUG "libmission_planner_component.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS drone_ignition_bringup::mission_planner_component )
list(APPEND _IMPORT_CHECK_FILES_FOR_drone_ignition_bringup::mission_planner_component "${_IMPORT_PREFIX}/lib/libmission_planner_component.so" )

# Import target "drone_ignition_bringup::drone_controller_enhanced_component" for configuration "Debug"
set_property(TARGET drone_ignition_bringup::drone_controller_enhanced_component APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(drone_ignition_bringup::drone_controller_enhanced_component PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libdrone_controller_enhanced_component.so"
  IMPORTED_SONAME_DEBUG "libdrone_controller_enhanced_component.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS drone_ignition_bringup::drone_controller_enhanced_component )
list(APPEND _IMPORT_CHECK_FILES_FOR_drone_ignition_bringup::drone_controller_enhanced_component "${_IMPORT_PREFIX}/lib/libdrone_controller_enhanced_component.so" )

# Import target "drone_ignition_bringup::pid_controller" for configuration "Debug"
set_property(TARGET drone_ignition_bringup::pid_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(drone_ignition_bringup::pid_controller PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libpid_controller.so"
  IMPORTED_SONAME_DEBUG "libpid_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS drone_ignition_bringup::pid_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_drone_ignition_bringup::pid_controller "${_IMPORT_PREFIX}/lib/libpid_controller.so" )

# Import target "drone_ignition_bringup::drone_control" for configuration "Debug"
set_property(TARGET drone_ignition_bringup::drone_control APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(drone_ignition_bringup::drone_control PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libdrone_control.so"
  IMPORTED_SONAME_DEBUG "libdrone_control.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS drone_ignition_bringup::drone_control )
list(APPEND _IMPORT_CHECK_FILES_FOR_drone_ignition_bringup::drone_control "${_IMPORT_PREFIX}/lib/libdrone_control.so" )

# Import target "drone_ignition_bringup::state_machine" for configuration "Debug"
set_property(TARGET drone_ignition_bringup::state_machine APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(drone_ignition_bringup::state_machine PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libstate_machine.so"
  IMPORTED_SONAME_DEBUG "libstate_machine.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS drone_ignition_bringup::state_machine )
list(APPEND _IMPORT_CHECK_FILES_FOR_drone_ignition_bringup::state_machine "${_IMPORT_PREFIX}/lib/libstate_machine.so" )

# Import target "drone_ignition_bringup::path_planner" for configuration "Debug"
set_property(TARGET drone_ignition_bringup::path_planner APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(drone_ignition_bringup::path_planner PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libpath_planner.so"
  IMPORTED_SONAME_DEBUG "libpath_planner.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS drone_ignition_bringup::path_planner )
list(APPEND _IMPORT_CHECK_FILES_FOR_drone_ignition_bringup::path_planner "${_IMPORT_PREFIX}/lib/libpath_planner.so" )

# Import target "drone_ignition_bringup::mission_executor" for configuration "Debug"
set_property(TARGET drone_ignition_bringup::mission_executor APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(drone_ignition_bringup::mission_executor PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmission_executor.so"
  IMPORTED_SONAME_DEBUG "libmission_executor.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS drone_ignition_bringup::mission_executor )
list(APPEND _IMPORT_CHECK_FILES_FOR_drone_ignition_bringup::mission_executor "${_IMPORT_PREFIX}/lib/libmission_executor.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
