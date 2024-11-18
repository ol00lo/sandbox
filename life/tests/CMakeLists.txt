#catch2
FetchContent_Declare(
  catch
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v2.13.10
)
FetchContent_MakeAvailable(catch)
set(CATCH2_FRAMEWORK "Catch2::Catch2")

add_executable(simlife_tests
	tests.cpp
	main.cpp)

target_include_directories(simlife_tests PRIVATE ${CMAKE_SOURCE_DIR}/lib)
target_link_libraries(simlife_tests PRIVATE Catch2::Catch2)
target_link_libraries(simlife_tests PUBLIC simlife_lib)

add_custom_command(TARGET simlife_tests POST_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/test_data/file.txt ${CMAKE_BINARY_DIR})
