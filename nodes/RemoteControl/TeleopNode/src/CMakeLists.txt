

add_library(baseRCWindow BaseWindow.cpp)
target_link_libraries(baseRCWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} commonWindowUtility)
add_dependencies(baseRCWindow eros_generate_messages_cpp)

add_subdirectory(MainWindow)
#add_subdirectory(test)