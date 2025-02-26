add_library(messageWindow MessageWindow.cpp)
target_link_libraries(messageWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} baseWindow)
add_dependencies(messageWindow eros_generate_messages_cpp)
add_subdirectory(test)