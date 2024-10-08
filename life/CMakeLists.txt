cmake_minimum_required(VERSION 3.10)
project(simlife)
set(CMAKE_CXX_STANDARD 17)

include(FindPython3)
if (NOT PYTHON3_FOUND)
    message(FATAL_ERROR "NO Python found!")
else()
    message(STATUS "PYTHON FOUND")
endif()

set(SRC 
    main.cpp
	game.cpp
	board.cpp
)
set(HEADERS
	game.hpp
	board.hpp
)

add_executable(simlife ${SRC} ${HEADERS})

add_custom_target(clangformat
    COMMAND ${PYTHON3_EXECUTABLE} clangformat.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
