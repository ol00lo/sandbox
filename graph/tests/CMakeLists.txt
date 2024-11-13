set (SRC
	main.cpp
	tests.cpp
)

add_executable(graph_pr ${SRC} ${HEADERS})
target_link_libraries(graph_pr PRIVATE Catch2::Catch2 graph_lib)
#target_include_directories(graph_pr PRIVATE ${CMAKE_SOURCE_DIR}/src/test)
