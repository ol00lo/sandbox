cmake_minimum_required(VERSION 3.10)
project(simlife)

set(CMAKE_CXX_STANDARD 17)

set(SRC 
    main.cpp
	game.cpp
	board.cpp
	simlife.cpp
)
set(HEADERS
	game.hpp
	board.hpp
	cxxopts.hpp
	simlife.hpp
)

add_executable(simlife ${SRC} ${HEADERS})
