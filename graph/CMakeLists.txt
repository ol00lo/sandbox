cmake_minimum_required(VERSION 3.10)
project(graph_pr)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

add_subdirectory(lib)
add_subdirectory(tests)