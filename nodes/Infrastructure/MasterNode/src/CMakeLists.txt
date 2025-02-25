add_library(masterNodeProcess MasterNodeProcess.cpp)
target_link_libraries(masterNodeProcess ${catkin_LIBRARIES} baseNodeProcess)

add_executable(master_node MasterNode.cpp) 
target_link_libraries(master_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} masterNodeProcess baseNode )
add_dependencies(master_node ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
