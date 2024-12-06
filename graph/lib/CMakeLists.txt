FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.12.0
)
FetchContent_MakeAvailable(spdlog)
FetchContent_Declare(
    json 
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)
add_library(graph_lib STATIC
  graph.cpp
  i_node.cpp
  input_node.cpp
  i_functional_node.cpp
  arithmetic_nodes.cpp
  power_nodes.cpp
  model.cpp
)
target_link_libraries(graph_lib PUBLIC spdlog::spdlog nlohmann_json::nlohmann_json)
target_include_directories(graph_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})