add_library(teleopJoyNodeProcess TeleopJoyNodeProcess.cpp)
target_link_libraries(teleopJoyNodeProcess ${catkin_LIBRARIES} baseNodeProcess)

add_executable(teleop_joy_node TeleopJoyNode.cpp) 
target_link_libraries(teleop_joy_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} teleopJoyNodeProcess baseNode )
add_dependencies(teleop_joy_node ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
