add_subdirectory(thin_kobuki_library)

add_executable(thin_kobuki_node thin_kobuki_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(thin_kobuki_node
  thin_kobuki_library
  ${catkin_LIBRARIES}
)
