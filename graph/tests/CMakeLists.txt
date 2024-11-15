set (SRC
	main.cpp
	tests.cpp
)

add_executable(graph_pr ${SRC} ${HEADERS})
target_link_libraries(graph_pr PRIVATE Catch2::Catch2 spdlog::spdlog graph_lib)