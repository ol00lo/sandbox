#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <pybind11/stl.h>
#include "tensor.hpp"
#include "data_node.hpp"
#include "i_node.hpp"
#include "model.hpp"

using namespace g;

PYBIND11_MODULE(py_graph_pr, m) {
    // Tensor
    py::class_<Tensor>(m, "Tensor")
        .def(py::init<>())
        .def(py::init<const std::vector<double>&>())
        .def(py::init<Shape,double>())
        .def(py::init<Shape,const std::vector<double>&>())

        .def_property_readonly("shape",
            [](const Tensor& t){
                auto s = t.shape();
                return std::vector<size_t>(s.begin(), s.end());
            }
        )
        .def_property_readonly("data",  &Tensor::data)

        .def("set_zero", &Tensor::set_zero)
        .def("add", &Tensor::add)
        .def("sub", &Tensor::sub)
        .def("mult", &Tensor::mult)
        .def("div", &Tensor::div)
        .def("scalar_mult", &Tensor::scalar_mult)
        .def("__repr__", [](const Tensor& t) {
            std::ostringstream oss; t.write(oss); return oss.str();
        });

    //INode
    py::class_<g::INode, std::shared_ptr<g::INode>>(m, "INode")
        .def("value", &g::INode::value)
        .def_static("factory",
            [](const std::string &classname,
               const std::string &nodename) {
                 return g::INode::factory(classname, nodename);
            },
            py::arg("classname"), py::arg("nodename") = "");

    // DataNode
    py::class_<g::DataNode, g::INode, std::shared_ptr<g::DataNode>>(m, "DataNode")
        .def(py::init<const std::string &>())
        .def("set_value", &g::DataNode::set_value)
        .def("value", &g::DataNode::value);

    m.def("set_dep",
          [](std::shared_ptr<g::INode> node,
             const std::vector<std::shared_ptr<g::INode>>& deps)
          {
              switch (deps.size()) {
                  case 0: g::set_dep(node, {}); break;
                  case 1: g::set_dep(node, { deps[0] }); break;
                  case 2: g::set_dep(node, { deps[0], deps[1] }); break;
                  case 3: g::set_dep(node, { deps[0], deps[1], deps[2] }); break;
                  default: _THROW_NOT_IMP_; break;
              }
          });
}