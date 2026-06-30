                
catkin_add_gtest(test_definitions test_definitions.cpp)
target_link_libraries(test_definitions ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_definitions eros_generate_messages_cpp)


catkin_add_gtest(test_logger test_logger.cpp)
target_link_libraries(test_logger ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger)
add_dependencies(test_logger eros_generate_messages_cpp)