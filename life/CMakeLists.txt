cmake_minimum_required(VERSION 3.10)
project(simlife)
set(CMAKE_CXX_STANDARD 17)

find_package(Python3 REQUIRED)
set(SRC 
    src/main.cpp
	src/game.cpp
	src/board.cpp
)
set(HEADERS
	src/game.hpp
	src/board.hpp
)
set(FORMAT_SOURCE_DIRS  sources headers)

add_executable(simlife ${SRC} ${HEADERS})

add_custom_target(clangformat
    COMMAND ${PYTHON3_EXECUTABLE} clangformat.py ${FORMAT_SOURCE_DIRS}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
