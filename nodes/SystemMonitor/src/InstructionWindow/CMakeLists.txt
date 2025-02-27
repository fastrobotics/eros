add_library(instructionWindow InstructionWindow.cpp)
target_link_libraries(instructionWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} baseWindow)
add_dependencies(instructionWindow eros_generate_messages_cpp)
#add_subdirectory(test)