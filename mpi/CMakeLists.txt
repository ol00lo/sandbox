cmake_minimum_required(VERSION 3.10)
project(solver_mpi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(MPI REQUIRED)
include_directories(SYSTEM ${MPI_INCLUDE_PATH})

set(SRC 
    Application6.cpp
    timer.cpp
)
set(HEADERS
    timer.hpp
)

add_executable(solver_mpi ${SRC} ${HEADERS})

target_link_libraries(solver_mpi PUBLIC MPI::MPI_CXX)