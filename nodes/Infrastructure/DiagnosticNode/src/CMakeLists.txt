add_library(diagnosticNodeProgress DiagnosticNodeProcess.cpp)
target_link_libraries(diagnosticNodeProgress ${catkin_LIBRARIES} baseNodeProcess)

add_executable(diagnostic_node DiagnosticNode.cpp) 
target_link_libraries(diagnostic_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} diagnosticNodeProgress baseNode )
add_dependencies(diagnostic_node ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
