#// clang-format off
add_library(systemMonitorProcess SystemMonitorProcess.cpp)
target_link_libraries(systemMonitorProcess ${catkin_LIBRARIES} ${CURSES_LIBRARIES} baseNodeProcess 
    baseWindow
    headerWindow 
    deviceWindow
    nodeWindow
    instructionWindow
    messageWindow
    statusWindow
    diagnosticsWindow)

add_executable(system_monitor SystemMonitorNode.cpp)
target_link_libraries(system_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CURSES_LIBRARIES} systemMonitorProcess baseNode)
add_dependencies(system_monitor ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
