cmake_minimum_required(VERSION 3.10)
project(graph_pr)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  catch
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v2.13.10
)
FetchContent_MakeAvailable(catch)
set(CATCH2_FRAMEWORK "Catch2::Catch2")

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.12.0
)
FetchContent_MakeAvailable(spdlog)

add_subdirectory(lib)
add_subdirectory(tests)