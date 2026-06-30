add_rostest_gtest(test_diagnosticsWindow
    test_diagnosticsWindow.test
    test_diagnosticsWindow.cpp
)
add_dependencies(test_diagnosticsWindow diagnosticsWindow)
target_link_libraries(test_diagnosticsWindow ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger diagnosticsWindow)