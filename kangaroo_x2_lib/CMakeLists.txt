cmake_minimum_required(VERSION 3.5)
project(kangaroo_x2_lib)

##################################################################
# Project Options
option(BUILD_TESTING "Build tests" OFF)
option(BUILD_DEMO "Build demos" ON)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
##################################################################

##################################################################
# Modes

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

add_definitions(-Werror=return-type)
##################################################################

##################################################################
# LIBRARIES
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)
find_package(LibSerial REQUIRED)

set(EXT_LIBS
    ${LibSerial_LIBRARY}
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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools.cpp
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
# BUILD
add_library(${PROJECT_NAME}
    ${LIB_INC}
    ${LIB_SRC}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${EXT_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${LibSerial_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
##################################################################

##################################################################
# Build demos
if(BUILD_DEMO)
    add_subdirectory(examples)
endif()
##################################################################

##################################################################
# Build tests
if(BUILD_TESTS)
    add_subdirectory(test)
endif()
##################################################################

##################################################################
# Install 
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
##################################################################