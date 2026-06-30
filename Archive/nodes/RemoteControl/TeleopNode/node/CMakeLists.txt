add_library(teleopNodeProcess TeleopNodeProcess.cpp)
target_link_libraries(teleopNodeProcess ${catkin_LIBRARIES} baseNodeProcess headerWindow mainRCWindow messageWindow diagnosticsWindow instructionRCWindow)

add_executable(teleop_gui TeleopNode.cpp) 
target_link_libraries(teleop_gui ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${rosbag_LIBRARIES} teleopNodeProcess baseNode )
add_dependencies(teleop_gui ${eros_EXPORTED_TARGETS})
add_subdirectory(test)