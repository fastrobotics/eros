catkin_add_gtest(test_diagnosticNodeProgress test_DiagnosticNodeProcess.cpp)
target_link_libraries(test_diagnosticNodeProgress ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger diagnosticNodeProgress)
add_dependencies(test_diagnosticNodeProgress eros_generate_messages_cpp)

add_rostest_gtest(test_diagnosticNode
    test_diagnosticNode.test
    test_diagnosticNode.cpp
)
add_dependencies(test_diagnosticNode diagnostic_node)
target_link_libraries(test_diagnosticNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)