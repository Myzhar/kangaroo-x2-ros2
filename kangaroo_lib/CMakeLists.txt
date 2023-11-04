cmake_minimum_required(VERSION 3.5)
project(kangaroo_x2_lib)

##################################################################
# Project Options
option(BUILD_TESTING "Build tests" OFF)
option(STATIC_CHECK "Run static check" OFF)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(STATIC_CHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        message(STATUS "Found cppcheck")
        set(CMAKE_CXX_CPPCHECK cppcheck;--std=c++11;--enable=all)
    endif() 
endif()
##################################################################

##################################################################
# Modes

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS " * Release Mode")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    message(STATUS " * Release with Debug Info Mode")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(STATUS " * Debug Mode")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
##################################################################

##################################################################
# LIBRARIES
set(EXT_LIBS
)

##################################################################
# SOURCES

set(LIB_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/types.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tools.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Stream.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/KangarooCRC.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/KangarooCommandWriter.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/KangarooReplyReceiver.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/KangarooReplyReader.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/KangarooStatus.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/KangarooSerial.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/KangarooTimeout.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/KangarooChannel.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/KangarooMonitor.hpp
)

set(LIB_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Stream.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KangarooCRC.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KangarooCommandWriter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KangarooReplyReceiver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KangarooReplyReader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KangarooStatus.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KangarooSerial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KangarooTimeout.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KangarooChannel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KangarooMonitor.cpp
)
##################################################################

##################################################################
# Build tests
if(BUILD_TESTS)
    add_subdirectory(test)
endif()
##################################################################

##################################################################
# BUILD
add_library(${PROJECT_NAME}
    ${LIB_INC}
    ${LIB_SRC}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${EXT_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src
)
##################################################################

##################################################################
# Install 
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
##################################################################