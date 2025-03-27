catkin_add_gtest(test_{{cookiecutter.class_binary}} test_{{cookiecutter.class_name}}.cpp)
target_link_libraries(test_{{cookiecutter.class_binary}} ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger {{cookiecutter.class_binary}})
add_dependencies(test_{{cookiecutter.class_binary}} eros_generate_messages_cpp)
