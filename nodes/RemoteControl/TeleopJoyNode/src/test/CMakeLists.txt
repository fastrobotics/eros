catkin_add_gtest(test_TeleopJoyNodeProcess test_teleopJoyNodeProcess.cpp)
target_link_libraries(test_TeleopJoyNodeProcess ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger TeleopJoyNodeProcess)
add_dependencies(test_TeleopJoyNodeProcess eros_generate_messages_cpp)

add_rostest_gtest(test_teleopJoyNode
    test_teleopJoyNode.test
    test_teleopJoyNode.cpp
)
add_dependencies(test_teleopJoyNode safety_node)
target_link_libraries(test_teleopJoyNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger baseNode)