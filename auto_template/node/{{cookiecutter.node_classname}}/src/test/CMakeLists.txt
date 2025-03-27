catkin_add_gtest(test_{{cookiecutter.process_name_binary}} test_{{cookiecutter.process_classname}}.cpp)
target_link_libraries(test_{{cookiecutter.process_name_binary}} ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger {{cookiecutter.process_name_binary}})
add_dependencies(test_{{cookiecutter.process_name_binary}} eros_generate_messages_cpp)

add_rostest_gtest(test_{{cookiecutter.node_classname}}
    test_{{cookiecutter.node_classname}}.test
    test_{{cookiecutter.node_classname}}.cpp
)
add_dependencies(test_{{cookiecutter.node_classname}} {{cookiecutter.node_name_binary}})
target_link_libraries(test_{{cookiecutter.node_classname}} ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)
