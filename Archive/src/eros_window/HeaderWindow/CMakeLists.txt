add_library(headerWindow HeaderWindow.cpp)
target_link_libraries(headerWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} baseWindow)
add_dependencies(headerWindow eros_generate_messages_cpp)
add_subdirectory(test)