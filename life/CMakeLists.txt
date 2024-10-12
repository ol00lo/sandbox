cmake_minimum_required(VERSION 3.10)
project(simlife)

set(CMAKE_CXX_STANDARD 17)
include(FetchContent)
FetchContent_Declare(
  cxxopts
  GIT_REPOSITORY https://github.com/jarro2783/cxxopts
  GIT_TAG        v3.2.0
)

FetchContent_MakeAvailable(cxxopts)
FetchContent_GetProperties(cxxopts)
if(NOT cxxopts_POPULATED)
	FetchContent_Populate(cxxopts)
endif()
include_directories(${cxxopts_SOURCE_DIR}/include)

set(SRC 
    main.cpp
	game.cpp
	board.cpp
	simlife.cpp
)
set(HEADERS
	game.hpp
	board.hpp
	simlife.hpp
)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

add_executable(simlife ${SRC} ${HEADERS})

add_custom_command(TARGET simlife POST_BUILD 
	COMMAND ${CMAKE_COMMAND} -E copy
		${CMAKE_SOURCE_DIR}/test_data/file.txt ${CMAKE_BINARY_DIR}/bin)