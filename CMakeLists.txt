#######################################
#       Project Configuration         #
#######################################

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(harmony_research_interface_torque_lib LANGUAGES CXX)

# Install folder
set(CMAKE_INSTALL_PREFIX $ENV{HOME}/harmony_research)
# Log folder
file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/bin/log)

#######################################
#      Resources Configuration        #
#######################################

include_directories(resources/eigen_library)
include_directories(resources/plog/include)

include_directories(resources/file_handling/include)
include_directories(resources/file_handling/include/json/single_include/nlohmann)

include_directories(include)

#######################################
#        Library Configuration        #
#######################################

set(CMAKE_LIBRARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
message(STATUS "Library path: " ${CMAKE_LIBRARY_PATH})

find_library(HARMONY_RESEARCH_LIB harmony_research)
if(HARMONY_RESEARCH_LIB)
    message(STATUS "Harmony research library found at: ${HARMONY_RESEARCH_LIB}")
    # Add imported library
    add_library(harmony_research STATIC IMPORTED)
    set_target_properties(harmony_research PROPERTIES IMPORTED_LOCATION ${HARMONY_RESEARCH_LIB})
else()
    message(STATUS "Harmony research library not found")
endif()

# File Handling
find_library(FILE_HANDLING_LIB file_handling)
if(FILE_HANDLING_LIB)
    message(STATUS "File handling library found at: ${FILE_HANDLING_LIB}")
    # Add imported library
    add_library(file_handling STATIC IMPORTED)
    set_target_properties(file_handling PROPERTIES IMPORTED_LOCATION ${FILE_HANDLING_LIB})
else()
    message(STATUS "File handling library not found")
    file(GLOB FILE_HANDLING_SOURCES resources/file_handling/src/*.cpp)
    file(GLOB FILE_HANDLING_HEADERS resources/file_handling/include/*.h)
    # Compile library from source
    add_library(file_handling STATIC
        ${FILE_HANDLING_SOURCES}
        ${FILE_HANDLING_HEADERS}
        )
    # Install targets
    install(TARGETS file_handling DESTINATION ${PROJECT_BINARY_DIR}/lib)
endif()

#######################################
#       Folder Configuration          #
#######################################

add_subdirectory(application)
add_subdirectory(tools)


