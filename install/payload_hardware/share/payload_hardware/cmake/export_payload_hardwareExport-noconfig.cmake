#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "payload_hardware::payload_hardware" for configuration ""
set_property(TARGET payload_hardware::payload_hardware APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(payload_hardware::payload_hardware PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libpayload_hardware.so"
  IMPORTED_SONAME_NOCONFIG "libpayload_hardware.so"
  )

list(APPEND _cmake_import_check_targets payload_hardware::payload_hardware )
list(APPEND _cmake_import_check_files_for_payload_hardware::payload_hardware "${_IMPORT_PREFIX}/lib/libpayload_hardware.so" )

# Import target "payload_hardware::payload_can_core" for configuration ""
set_property(TARGET payload_hardware::payload_can_core APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(payload_hardware::payload_can_core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libpayload_can_core.a"
  )

list(APPEND _cmake_import_check_targets payload_hardware::payload_can_core )
list(APPEND _cmake_import_check_files_for_payload_hardware::payload_can_core "${_IMPORT_PREFIX}/lib/libpayload_can_core.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
