catkin_add_gtest(test_safetyNodeProcess test_SafetyNodeProcess.cpp)
target_link_libraries(test_safetyNodeProcess ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger safetyNodeProcess armedStateManager)
add_dependencies(test_safetyNodeProcess eros_generate_messages_cpp)

add_rostest_gtest(test_safetyNode
    test_safetyNode.test
    test_safetyNode.cpp
)
add_dependencies(test_safetyNode safety_node)
target_link_libraries(test_safetyNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger baseNode)