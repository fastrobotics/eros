add_library(mainRCWindow MainWindow.cpp)
target_link_libraries(mainRCWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} baseRCWindow)
add_dependencies(mainRCWindow eros_generate_messages_cpp)
#add_subdirectory(test)