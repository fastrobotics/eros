add_library(sampleClass SampleClass.cpp)
target_link_libraries(sampleClass ${catkin_LIBRARIES} logger)
add_dependencies(sampleClass eros_generate_messages_cpp)
add_subdirectory(test)
