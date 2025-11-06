#######################################
#      File Handling Library          #
#######################################

cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME file_handling)
project(${PROJECT_NAME})

message(STATUS "Ready to build ${PROJECT_NAME}")

# Find system-installed Eigen3 (required)
find_package(Eigen3 REQUIRED)
message(STATUS "Using system-installed Eigen3")
set(EIGEN_TARGET Eigen3::Eigen)

# Helper function to link dependencies to a target
function(link_file_handling_dependencies target_name visibility)
    # Link Eigen (system-installed)
    target_link_libraries(${target_name} ${visibility} ${EIGEN_TARGET})
    
    # Link plog
    if(TARGET plog)
        target_link_libraries(${target_name} ${visibility} plog)
    else()
        target_include_directories(${target_name} ${visibility}
            ${CMAKE_CURRENT_SOURCE_DIR}/../plog/include
        )
    endif()
endfunction()

# Try to find pre-built library first
find_library(FILE_HANDLING_LIB file_handling
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../lib
    NO_DEFAULT_PATH
)

if(FILE_HANDLING_LIB)
    message(STATUS "File handling library found at: ${FILE_HANDLING_LIB}")
    
    # Add imported library
    add_library(file_handling STATIC IMPORTED)
    set_target_properties(file_handling PROPERTIES IMPORTED_LOCATION ${FILE_HANDLING_LIB})
    
    target_include_directories(file_handling INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/json/single_include/nlohmann
    )
    
    link_file_handling_dependencies(file_handling INTERFACE)
else()
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
    )
    
    # Link dependencies
    link_file_handling_dependencies(file_handling PUBLIC)
    
    # Install targets
    install(TARGETS file_handling DESTINATION lib)
    install(DIRECTORY include/ DESTINATION include
        FILES_MATCHING PATTERN "*.h"
    )
endif()
