# Commenting out for now
add_library(snapshotProcess SnapshotProcess.cpp)
target_link_libraries(snapshotProcess ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} baseNodeProcess coreUtility)

add_executable(snapshot_node SnapshotNode.cpp) 
target_link_libraries(snapshot_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${Boost_LIBRARIES} snapshotProcess baseNode )
add_dependencies(snapshot_node ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
