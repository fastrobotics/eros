add_library(safetyNodeProcess SafetyNodeProcess.cpp)
target_link_libraries(safetyNodeProcess ${catkin_LIBRARIES} baseNodeProcess armedStateManager)

add_executable(safety_node SafetyNode.cpp) 
target_link_libraries(safety_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} safetyNodeProcess baseNode )
add_dependencies(safety_node ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
