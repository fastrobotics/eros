catkin_add_gtest(test_sampleClass test_SampleClass.cpp)
target_link_libraries(test_sampleClass ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger sampleClass)
add_dependencies(test_sampleClass eros_generate_messages_cpp)
