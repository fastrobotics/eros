add_subdirectory(eros_utility)
add_subdirectory(eros_diagnostic)
add_subdirectory(eros_window)


add_library(logger Logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES} diagnosticUtility)
add_dependencies(logger eros_generate_messages_cpp)

add_library(resourceMonitor ResourceMonitor.cpp)
target_link_libraries(resourceMonitor ${catkin_LIBRARIES} coreUtility)
add_dependencies(resourceMonitor eros_generate_messages_cpp)

add_library(armedStateManager ArmedStateManager.cpp)
target_link_libraries(armedStateManager ${catkin_LIBRARIES} diagnosticUtility)
add_dependencies(armedStateManager ${eros_EXPORTED_TARGETS})

add_library(baseNodeProcess BaseNodeProcess.cpp)
target_link_libraries(baseNodeProcess ${catkin_LIBRARIES} coreUtility convertUtility logger diagnosticUtility diagnosticManager)
add_dependencies(baseNodeProcess eros_generate_messages_cpp)

add_library(baseNode BaseNode.cpp)
target_link_libraries(baseNode ${catkin_LIBRARIES} baseNodeProcess logger resourceMonitor)
add_dependencies(baseNode eros_generate_messages_cpp)

add_subdirectory(test)

install(TARGETS logger baseNodeProcess baseNode
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
