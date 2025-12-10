# Get application name from folder
get_filename_component(THIS_APP_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "${THIS_APP_NAME} application is ready to build")

# Get source and header files
file(GLOB THIS_APP_SOURCES *.cpp)
file(GLOB THIS_APP_HEADERS *.h)

# Create executables and link libraries
add_executable(${THIS_APP_NAME} ${THIS_APP_SOURCES} ${THIS_APP_HEADERS})
find_package(Threads REQUIRED)
target_link_libraries(${THIS_APP_NAME} 
    PRIVATE
        harmony_research
        file_handling  # Propagates eigen and plog
        Threads::Threads
)

# Install targets
install(TARGETS ${THIS_APP_NAME} DESTINATION bin)
