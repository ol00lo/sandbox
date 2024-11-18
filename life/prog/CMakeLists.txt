#cxxopts
FetchContent_Declare(
  cxxopts
  GIT_REPOSITORY https://github.com/jarro2783/cxxopts
  GIT_TAG        v3.2.0
)
FetchContent_MakeAvailable(cxxopts)
include_directories(${cxxopts_SOURCE_DIR}/include)

add_executable(simlife 
	main.cpp 
	arguments.cpp
	simlife_utils.cpp)

target_link_libraries(simlife PRIVATE cxxopts)
target_link_libraries(simlife PUBLIC simlife_lib)

add_custom_command(TARGET simlife POST_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/test_data/file.txt ${CMAKE_BINARY_DIR})
