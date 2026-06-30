catkin_add_gtest(test_masterNodeProcess test_MasterNodeProcess.cpp)
target_link_libraries(test_masterNodeProcess ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger masterNodeProcess)
add_dependencies(test_masterNodeProcess eros_generate_messages_cpp)

add_rostest_gtest(test_masterNode
    test_masterNode.test
    test_masterNode.cpp
)
add_dependencies(test_masterNode master_node)
target_link_libraries(test_masterNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger baseNode)