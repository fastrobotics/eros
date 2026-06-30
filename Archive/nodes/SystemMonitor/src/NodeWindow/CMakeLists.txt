add_library(nodeWindow NodeWindow.cpp)
target_link_libraries(nodeWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} baseWindow)
add_dependencies(nodeWindow eros_generate_messages_cpp)
add_subdirectory(test)