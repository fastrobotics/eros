add_rostest_gtest(test_teleopnode_process
    test_teleopnode_process.test
    test_teleopnode_process.cpp
)
add_dependencies(test_teleopnode_process eros_generate_messages_cpp)
target_link_libraries(test_teleopnode_process ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger teleopNodeProcess)

add_rostest_gtest(test_teleop_node
    test_teleop_node.test
    test_teleop_node.cpp
)
add_dependencies(test_teleop_node eros_generate_messages_cpp)
target_link_libraries(test_teleop_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)