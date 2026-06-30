add_rostest_gtest(test_nodeWindow
    test_nodeWindow.test
    test_nodeWindow.cpp
)
add_dependencies(test_nodeWindow diagnosticsWindow)
target_link_libraries(test_nodeWindow ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger nodeWindow)