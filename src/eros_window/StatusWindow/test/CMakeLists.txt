catkin_add_gtest(test_statusWindow test_statusWindow.cpp)
target_link_libraries(test_statusWindow ${catkin_LIBRARIES} ${Boost_LIBRARIES} statusWindow logger)
add_dependencies(test_statusWindow eros_generate_messages_cpp)