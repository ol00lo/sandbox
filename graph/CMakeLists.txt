cmake_minimum_required(VERSION 3.10)
project(graph_pr)
set(CMAKE_CXX_STANDARD 17)

include_directories("${CMAKE_SOURCE_DIR}/extern/catch2")

add_library(graph STATIC graph.cpp functional_nodes.cpp op.cpp)
target_include_directories(graph PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_executable(main main.cpp tests.cpp)
target_link_libraries(main PRIVATE graph)