add_library(statusWindow StatusWindow.cpp)
target_link_libraries(statusWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} baseWindow)
add_dependencies(statusWindow eros_generate_messages_cpp)
add_subdirectory(test)