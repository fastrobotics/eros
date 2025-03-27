catkin_add_gtest(test_sampleNodeProcess test_SampleNodeProcess.cpp)
target_link_libraries(test_sampleNodeProcess ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger sampleNodeProcess)
add_dependencies(test_sampleNodeProcess eros_generate_messages_cpp)

add_rostest_gtest(test_SampleNode
    test_SampleNode.test
    test_SampleNode.cpp
)
add_dependencies(test_SampleNode sample_node)
target_link_libraries(test_SampleNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)
