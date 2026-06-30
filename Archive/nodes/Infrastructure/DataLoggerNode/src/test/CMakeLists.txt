catkin_add_gtest(test_dataLoggerProcess test_DataLoggerProcess.cpp)
target_link_libraries(test_dataLoggerProcess ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger dataLoggerProcess)
add_dependencies(test_dataLoggerProcess eros_generate_messages_cpp)

add_rostest_gtest(test_dataLoggerNode
    test_dataLoggerNode.test
    test_dataLoggerNode.cpp
)
add_dependencies(test_dataLoggerNode datalogger_node)
target_link_libraries(test_dataLoggerNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)