FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.12.0
)
FetchContent_MakeAvailable(spdlog)

add_library(graph_lib STATIC
  graph.cpp
  i_node.cpp
  input_node.cpp
  i_functional_node.cpp
  arithmetic_nodes.cpp
  power_nodes.cpp
)
target_link_libraries(graph_lib PUBLIC spdlog::spdlog)
target_include_directories(graph_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})