cmake_minimum_required(VERSION 3.10)
project(simlife)
set(CMAKE_CXX_STANDARD 17)

#for clangformat
find_package(Python3 REQUIRED)

#cxxopts
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

#catch2
FetchContent_Declare(
  catch
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v2.13.10
)
FetchContent_GetProperties(catch)
if(NOT catch_POPULATED)
  FetchContent_Populate(catch)
  add_subdirectory(${catch_SOURCE_DIR} ${catch_BINARY_DIR})
endif()
set(CATCH2_FRAMEWORK "Catch2::Catch2")


#library
add_library(simlife_lib SHARED
	lib/board.cpp
	lib/simlife_utils.cpp
	lib/cmd_viewer.cpp
	lib/life_engine.cpp
	lib/driver.cpp
)
target_compile_definitions(simlife_lib PRIVATE LIFELIB_EXP)
target_include_directories(simlife_lib PRIVATE prog)

#exe
SET (SRC
	prog/main.cpp
	prog/arguments.cpp
	prog/tests/tests.cpp
)
add_executable(simlife ${SRC})
target_link_libraries(simlife PRIVATE Catch2::Catch2 simlife_lib)

#custom
set(FORMAT_SOURCE_DIRS  prog)
add_custom_command(TARGET simlife POST_BUILD 
	COMMAND ${CMAKE_COMMAND} -E copy
		${CMAKE_SOURCE_DIR}/test_data/file.txt ${CMAKE_BINARY_DIR})
add_custom_target(clangformat
    COMMAND ${PYTHON3_EXECUTABLE} clangformat.py ${FORMAT_SOURCE_DIRS}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
target_include_directories(simlife PRIVATE prog)
target_include_directories(simlife PRIVATE lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)