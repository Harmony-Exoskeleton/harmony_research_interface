#######################################
#       Project Configuration         #
#######################################

cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(harmony_research_interface_torque_lib LANGUAGES CXX)

# Install folder
set(CMAKE_INSTALL_PREFIX $ENV{HOME}/harmony_research)
# Log folder
file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/bin/log)

#######################################
#      Include Directories             #
#######################################

include_directories(include)

#######################################
#        Library Configuration        #
#######################################

set(CMAKE_LIBRARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
message(STATUS "Library path: " ${CMAKE_LIBRARY_PATH})

find_library(HARMONY_RESEARCH_LIB harmony_research
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
    NO_DEFAULT_PATH
)
if(HARMONY_RESEARCH_LIB)
    message(STATUS "Harmony research library found at: ${HARMONY_RESEARCH_LIB}")
    # Add imported library
    add_library(harmony_research STATIC IMPORTED)
    set_target_properties(harmony_research PROPERTIES IMPORTED_LOCATION ${HARMONY_RESEARCH_LIB})
else()
    message(STATUS "Harmony research library not found")
endif()

#######################################
#       Folder Configuration          #
#######################################

# Add resources subdirectory (handles all resource libraries)
add_subdirectory(resources)

# Add application and tools subdirectories
add_subdirectory(application)
add_subdirectory(tools)


