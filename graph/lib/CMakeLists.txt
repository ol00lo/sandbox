add_library(graph_lib STATIC
  graph.cpp
  i_node.cpp
  input_node.cpp
  i_functions_node.cpp
  arithmetic_nodes.cpp
  power_nodes.cpp
)
target_link_libraries(graph_lib PRIVATE spdlog::spdlog)
target_include_directories(graph_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})