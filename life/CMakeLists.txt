cmake_minimum_required(VERSION 3.10)
project(simlife)

set(CMAKE_CXX_STANDARD 17)

find_package(SFML COMPONENTS graphics window system REQUIRED)

set(SRC 
    main.cpp
	game.cpp
	board.cpp
	display.cpp
)
set(HEADERS
	game.hpp
	board.hpp
	display.hpp
)

add_executable(simlife ${SRC} ${HEADERS})

target_link_libraries(simlife PRIVATE sfml-graphics sfml-window sfml-system)

set(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
file(MAKE_DIRECTORY ${RESOURCE_DIR})
set(RESOURCES_FILES
	resources/cat.png
	resources/Samson.ttf
)
file(COPY ${RESOURCES_FILES} DESTINATION ${RESOURCE_DIR})