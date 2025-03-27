include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library({{cookiecutter.process_name_binary}} {{cookiecutter.process_classname}}.cpp)
target_link_libraries({{cookiecutter.process_name_binary}} ${catkin_LIBRARIES} baseNodeProcess)

add_executable({{cookiecutter.node_name_binary}} {{cookiecutter.node_classname}}.cpp) 
target_link_libraries({{cookiecutter.node_name_binary}} ${catkin_LIBRARIES} ${Boost_LIBRARIES} {{cookiecutter.process_name_binary}} baseNode )
add_dependencies({{cookiecutter.node_name_binary}} ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
