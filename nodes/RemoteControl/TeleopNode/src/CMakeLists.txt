add_library(remoteControlUtility RemoteControlUtility.cpp)
target_link_libraries(remoteControlUtility ${catkin_LIBRARIES} ${CURSES_LIBRARIES})
add_dependencies(remoteControlUtility eros_generate_messages_cpp)

add_library(baseRCWindow BaseWindow.cpp)
target_link_libraries(baseRCWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} remoteControlUtility)
add_dependencies(baseRCWindow eros_generate_messages_cpp)

add_subdirectory(MainWindow)
#add_subdirectory(test)