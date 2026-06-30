add_library(logger Logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES})
add_dependencies(logger eros_generate_messages_cpp)

add_library(baseNodeProcess BaseNodeProcess.cpp)
target_link_libraries(baseNodeProcess ${catkin_LIBRARIES} logger)
add_dependencies(baseNodeProcess eros_generate_messages_cpp)

add_library(baseNode BaseNode.cpp)
target_link_libraries(baseNode ${catkin_LIBRARIES} baseNodeProcess logger)
add_dependencies(baseNode eros_generate_messages_cpp)

add_subdirectory(test)

install(TARGETS logger baseNode 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
