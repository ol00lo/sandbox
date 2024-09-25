cmake_minimum_required(VERSION 3.10)
project(simlife)

set(CMAKE_CXX_STANDARD 17)

#set(SFML_DIR "c:/Users/mymri/bin/SFML-2.6.1/lib/cmake/SFML")
#find_package(SFML COMPONENTS graphics window system REQUIRED)

set(SRC 
    main.cpp
	game.cpp
	#display.cpp
)
set(HEADERS
	game.hpp
	#display.hpp
)

add_executable(simlife ${SRC} ${HEADERS})

#target_link_libraries(simlife sfml-graphics sfml-window sfml-system)