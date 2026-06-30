add_rostest_gtest(test_systemmonitor_process
    test_systemmonitor_process.test
    test_systemmonitor_process.cpp
)
add_dependencies(test_systemmonitor_process eros_generate_messages_cpp)
target_link_libraries(test_systemmonitor_process ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger systemMonitorProcess)

add_rostest_gtest(test_systemmonitor_node
    test_systemmonitor_node.test
    test_systemmonitor_node.cpp
)
add_dependencies(test_systemmonitor_node eros_generate_messages_cpp)
target_link_libraries(test_systemmonitor_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)