add_library(teleopNodeProcess TeleopNodeProcess.cpp)
target_link_libraries(teleopNodeProcess ${catkin_LIBRARIES} baseNodeProcess baseRCWindow)

add_executable(teleop_node TeleopNode.cpp) 
target_link_libraries(teleop_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${rosbag_LIBRARIES} teleopNodeProcess baseNode )
add_dependencies(teleop_node ${eros_EXPORTED_TARGETS})
#add_subdirectory(test)