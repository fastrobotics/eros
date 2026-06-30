#catkin_add_gtest(test_instructionWindow test_instructionwindow.cpp)
#target_link_libraries(test_instructionWindow ${catkin_LIBRARIES} ${Boost_LIBRARIES} instructionRCWindow logger)
#add_dependencies(test_instructionWindow eros_generate_messages_cpp)

add_rostest_gtest(test_instructionWindow
    test_instructionWindow.test
    test_instructionWindow.cpp
)
add_dependencies(test_instructionWindow instructionWindow)
target_link_libraries(test_instructionWindow ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger instructionRCWindow)