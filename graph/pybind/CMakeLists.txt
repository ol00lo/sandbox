find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.11.1
)
FetchContent_MakeAvailable(pybind11)

pybind11_add_module(py_graph_pr pybind_graph_pr.cpp)

target_include_directories(py_graph_pr PRIVATE ${PROJECT_SOURCE_DIR}/lib)
target_link_libraries(py_graph_pr PRIVATE graph_lib)

if (MSVC)
    target_link_options(py_graph_pr PRIVATE "/WHOLEARCHIVE:graph_lib")
else()
    target_link_options(py_graph_pr PRIVATE
        "-Wl,--whole-archive" graph_lib "-Wl,--no-whole-archive")
endif()
