# Locate LibSerial
#
# This module defines
#  LibSerial_FOUND, if false, do not try to link to LibSerial
#  LibSerial_LIBRARY, where to find LibSerial
#  LibSerial_INCLUDE_DIR, where to find SerialPort.h

# find the LibSerial include directory
find_path(LibSerial_INCLUDE_DIR 
          NAMES SerialPort.h
          PATH_SUFFIXES include libserial
          PATHS /usr)

# find the LibSerial library
find_library(LibSerial_LIBRARY
             NAMES libserial.so
             PATH_SUFFIXES lib
             PATHS /usr)

# handle the QUIETLY and REQUIRED arguments and set LibSerial_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LibSerial REQUIRED_VARS LibSerial_INCLUDE_DIR LibSerial_LIBRARY)
if(LibSerial_FOUND)
    mark_as_advanced(LibSerial_INCLUDE_DIR LibSerial_LIBRARY)
endif()
