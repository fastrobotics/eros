add_library(commonWindowUtility CommonWindowUtility.cpp)
target_link_libraries(commonWindowUtility ${catkin_LIBRARIES} ${CURSES_LIBRARIES})
add_dependencies(commonWindowUtility eros_generate_messages_cpp)

add_library(baseWindow BaseWindow.cpp)
target_link_libraries(baseWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} commonWindowUtility)
add_dependencies(baseWindow eros_generate_messages_cpp)


add_subdirectory(HeaderWindow)
add_subdirectory(DiagnosticsWindow)
add_subdirectory(MessageWindow)
add_subdirectory(StatusWindow)
add_subdirectory(test)