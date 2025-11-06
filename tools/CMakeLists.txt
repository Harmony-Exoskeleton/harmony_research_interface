get_filename_component(THIS_APP_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "${THIS_APP_NAME} folder, is ready to build")

add_executable(data_logger data_logger.cpp)
target_link_libraries(data_logger PRIVATE harmony_research)
install(TARGETS data_logger DESTINATION bin/tools)

add_executable(data_exerciser data_exerciser.cpp)
target_link_libraries(data_exerciser PRIVATE harmony_research)
install(TARGETS data_exerciser DESTINATION bin/tools)

add_executable(controller_printer controller_printer.cpp)
target_link_libraries(controller_printer PRIVATE harmony_research eigen)
install(TARGETS controller_printer DESTINATION bin/tools)

add_executable(set_controller set_controller.cpp)
target_link_libraries(set_controller PRIVATE harmony_research)
install(TARGETS set_controller DESTINATION bin/tools)

add_executable(stub_harmony stub_harmony.cpp)
target_link_libraries(stub_harmony PRIVATE harmony_research)
install(TARGETS stub_harmony DESTINATION bin/tools)

add_executable(value_printer value_printer.cpp)
target_link_libraries(value_printer PRIVATE harmony_research eigen)
install(TARGETS value_printer DESTINATION bin/tools)
