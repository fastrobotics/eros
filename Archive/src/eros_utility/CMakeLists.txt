add_library(coreUtility CoreUtility.cpp)
target_link_libraries(coreUtility ${catkin_LIBRARIES})
add_dependencies(coreUtility eros_generate_messages_cpp)

add_library(convertUtility ConvertUtility.cpp)
target_link_libraries(convertUtility ${catkin_LIBRARIES})
add_dependencies(convertUtility eros_generate_messages_cpp)

add_library(prettyUtility PrettyUtility.cpp)
target_link_libraries(prettyUtility ${catkin_LIBRARIES})
add_dependencies(prettyUtility eros_generate_messages_cpp)
add_subdirectory(test)