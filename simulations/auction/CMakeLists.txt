cmake_minimum_required(VERSION 3.10)
project(auction)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(include)

add_executable(auction 
	src/main.cpp
	src/game.cpp
	src/player.cpp
	src/task.cpp
)
