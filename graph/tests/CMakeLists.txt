FetchContent_Declare(
  catch
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v2.13.10
)
FetchContent_MakeAvailable(catch)
set(CATCH2_FRAMEWORK "Catch2::Catch2")

set (SRC
	main.cpp
	tests.cpp
	tensor_test.cpp
	generator_test.cpp
	curve_fitting_test.cpp
)

add_executable(graph_pr ${SRC} ${HEADERS})
target_link_libraries(graph_pr PRIVATE Catch2::Catch2 graph_lib)