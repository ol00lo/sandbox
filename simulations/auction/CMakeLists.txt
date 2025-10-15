cmake_minimum_required(VERSION 3.10)
project(auction)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(include)

add_executable(auction 
	src/main.cpp
	src/game.cpp
	src/awaiter.cpp
	src/task.cpp
)

find_program(CLANG_FORMAT clang-format)
if(CLANG_FORMAT)
    add_custom_target(format
        COMMAND ${CLANG_FORMAT} -i src/*.cpp include/*.h
        COMMENT "Formatting code..."
    )
endif()
