add_library({{cookiecutter.class_binary}} {{cookiecutter.class_name}}.cpp)
target_link_libraries({{cookiecutter.class_binary}} ${catkin_LIBRARIES} logger)
add_dependencies({{cookiecutter.class_binary}} eros_generate_messages_cpp)
add_subdirectory(test)
