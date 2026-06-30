#// clang-format off
project(eros_SnapshotNodeTest)
add_definitions(-DTESTDATA_DIR="${PROJECT_SOURCE_DIR}/data")
catkin_add_gtest(test_snapshotProcess test_SnapshotProcess.cpp)
target_link_libraries(test_snapshotProcess ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger snapshotProcess)
add_dependencies(test_snapshotProcess eros_generate_messages_cpp)

add_rostest_gtest(test_snapshotNode_Master
    test_snapshotNode_Master.test
    test_snapshotNode_Master.cpp
)
add_custom_command(
        TARGET test_snapshotNode_Master PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${PROJECT_SOURCE_DIR}/data/config/SnapshotConfig.xml
                ${CMAKE_CURRENT_BINARY_DIR}/config/SnapshotConfig.xml)

add_dependencies(test_snapshotNode_Master snapshot_node)
target_link_libraries(test_snapshotNode_Master ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)

add_rostest_gtest(test_snapshotNode_Slave
    test_snapshotNode_Slave.test
    test_snapshotNode_Slave.cpp
)
add_dependencies(test_snapshotNode_Slave snapshot_node)
target_link_libraries(test_snapshotNode_Slave ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger baseNodeProcess)