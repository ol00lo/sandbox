cmake_minimum_required(VERSION 3.10)
project(graph_pr)
set(CMAKE_CXX_STANDARD 17)

find_package(Python3 REQUIRED)
include(FetchContent)

add_subdirectory(lib)
add_subdirectory(tests)
add_subdirectory(python)

set(FORMAT_SOURCE_DIRS  lib tests)
add_custom_target(clangformat
    COMMAND ${PYTHON3_EXECUTABLE} clangformat.py ${FORMAT_SOURCE_DIRS}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)