#######################################
#      File Handling Library          #
#######################################

cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME file_handling)
project(${PROJECT_NAME})

message(STATUS "Ready to build ${PROJECT_NAME}")

# Define include directories for dependencies
set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../eigen)
set(PLOG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../plog/include)

# Try to find pre-built library first
find_library(FILE_HANDLING_LIB file_handling
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../lib
    NO_DEFAULT_PATH
)

if(FILE_HANDLING_LIB)
    message(STATUS "File handling library found at: ${FILE_HANDLING_LIB}")
endif()

#     # Add imported library
#     add_library(file_handling STATIC IMPORTED)
#     set_target_properties(file_handling PROPERTIES IMPORTED_LOCATION ${FILE_HANDLING_LIB})
    
#     target_include_directories(file_handling INTERFACE
#         ${CMAKE_CURRENT_SOURCE_DIR}/include
#         ${CMAKE_CURRENT_SOURCE_DIR}/include/json/single_include/nlohmann
#         ${EIGEN_INCLUDE_DIR}
#         ${PLOG_INCLUDE_DIR}
#     )
# else()
message(STATUS "File handling library not found, building from source")

# Get source and header files
file(GLOB FILE_HANDLING_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB FILE_HANDLING_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

# Compile library from source
add_library(file_handling STATIC
    ${FILE_HANDLING_SOURCES}
    ${FILE_HANDLING_HEADERS}
)

# Configure include directories
target_include_directories(file_handling PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/json/single_include/nlohmann>
    $<INSTALL_INTERFACE:include/json/single_include/nlohmann>
    $<BUILD_INTERFACE:${EIGEN_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${PLOG_INCLUDE_DIR}>
)

# Copy to project lib directory (main project /lib, not install folder)
add_custom_command(TARGET file_handling POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/lib
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:file_handling>
        ${CMAKE_SOURCE_DIR}/lib/
    COMMENT "Copying file_handling library to lib directory"
)

# Install targets (for system installation if needed)
# install(TARGETS file_handling DESTINATION lib)
