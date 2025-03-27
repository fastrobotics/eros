include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(sampleNodeProcess SampleNodeProcess.cpp)
target_link_libraries(sampleNodeProcess ${catkin_LIBRARIES} baseNodeProcess)

add_executable(sample_node SampleNode.cpp) 
target_link_libraries(sample_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} sampleNodeProcess baseNode )
add_dependencies(sample_node ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
