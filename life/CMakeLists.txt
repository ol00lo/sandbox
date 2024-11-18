cmake_minimum_required(VERSION 3.10)
project(simlife)
set(CMAKE_CXX_STANDARD 17)

#for clangformat
find_package(Python3 REQUIRED)
include(FetchContent)

# build all binaries to build/bin
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(lib)
add_subdirectory(prog)
add_subdirectory(tests)

#custom
set(FORMAT_SOURCE_DIRS  prog test)
add_custom_target(clangformat
    COMMAND ${PYTHON3_EXECUTABLE} clangformat.py ${FORMAT_SOURCE_DIRS}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)