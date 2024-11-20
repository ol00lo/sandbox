#cxxopts
FetchContent_Declare(
  cxxopts
  GIT_REPOSITORY https://github.com/jarro2783/cxxopts
  GIT_TAG        v3.2.0
)
FetchContent_MakeAvailable(cxxopts)

add_executable(simlife 
	main.cpp 
	arguments.cpp
	simlife_utils.cpp)
	
target_include_directories(simlife 
	PRIVATE ${cxxopts_SOURCE_DIR}/include
	PUBLIC ${CMAKE_SOURCE_DIR}/lib)
	
target_link_libraries(simlife 
	PRIVATE cxxopts
	PUBLIC simlife_lib)
	
add_custom_command(TARGET simlife POST_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/test_data/file.txt ${CMAKE_BINARY_DIR})
