cmake_minimum_required(VERSION 3.10)
project(simlife)
set(CMAKE_CXX_STANDARD 17)

find_package(Python3 REQUIRED)

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
	prog/sources/main.cpp
	prog/sources/board.cpp
	prog/sources/simlife_utils.cpp
	prog/sources/arguments.cpp
	prog/sources/cmd_viewer.cpp
	prog/sources/reg_engine.cpp
	prog/sources/driver.cpp
)
set(HEADERS
	prog/headers/board.hpp
	prog/headers/simlife_utils.hpp
	prog/headers/arguments.hpp
	prog/headers/cmd_viewer.hpp
	prog/headers/reg_engine.hpp
	prog/headers/engine.hpp
	prog/headers/viewer.hpp
	prog/headers/driver.hpp
)
include_directories(prog/headers)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

set(FORMAT_SOURCE_DIRS  prog)
add_executable(simlife ${SRC} ${HEADERS})

add_custom_command(TARGET simlife POST_BUILD 
	COMMAND ${CMAKE_COMMAND} -E copy
		${CMAKE_SOURCE_DIR}/test_data/file.txt ${CMAKE_BINARY_DIR})

add_custom_target(clangformat
    COMMAND ${PYTHON3_EXECUTABLE} clangformat.py ${FORMAT_SOURCE_DIRS}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
