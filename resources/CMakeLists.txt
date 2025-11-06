#######################################
#      Resources Configuration        #
#######################################

# Header-only libraries - create interface targets for proper include management
# Eigen library
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/eigen_library
)

# PLOG library
add_library(plog INTERFACE)
target_include_directories(plog INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/plog/include
)

# File Handling Library
# Compiled library with proper include directories and dependencies
add_subdirectory(file_handling)

# Rosbridge Client CPP library
# Compiled library with proper include directories and dependencies
# The rosbridge2cpp target provides headers and links against required libraries
add_subdirectory(rosbridge_client_cpp)
