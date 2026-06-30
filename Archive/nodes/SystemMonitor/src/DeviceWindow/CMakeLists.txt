add_library(deviceWindow DeviceWindow.cpp)
target_link_libraries(deviceWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} baseWindow)
add_dependencies(deviceWindow eros_generate_messages_cpp)
add_subdirectory(test)