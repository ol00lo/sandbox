cmake_minimum_required(VERSION 3.10)
project(graph_pr)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  catch
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v2.13.10
)
FetchContent_GetProperties(catch)
if(NOT catch_POPULATED)
  FetchContent_Populate(catch)
  add_subdirectory(${catch_SOURCE_DIR} ${catch_BINARY_DIR})
endif()
set(CATCH2_FRAMEWORK "Catch2::Catch2")

add_subdirectory(lib)
add_subdirectory(tests)