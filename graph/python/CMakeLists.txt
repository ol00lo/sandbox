find_package(pybind11 REQUIRED)

pybind11_add_module(py_graph_pr bindings.cpp)

target_link_libraries(py_graph_pr PRIVATE graph_lib)

target_include_directories(py_graph_pr PRIVATE ${PROJECT_SOURCE_DIR}/lib)
