#include "data_node.hpp"
#include "generator.hpp"
#include "i_node.hpp"
#include "model.hpp"
#include "optimization_worker.hpp"
#include "optimizer.hpp"
#include "tensor.hpp"
#include "training_graph.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace g;

PYBIND11_MODULE(py_graph_pr, m)
{
    // Tensor
    py::class_<Tensor>(m, "Tensor")
        .def(py::init<>())
        .def(py::init<const std::vector<double>&>())
        .def(py::init<Shape, double>())
        .def(py::init<Shape, const std::vector<double>&>())

        .def_property_readonly("shape",
                               [](const Tensor& t) {
                                   auto s = t.shape();
                                   return std::vector<size_t>(s.begin(), s.end());
                               })
        .def_property_readonly("data", &Tensor::data)
        .def("set_zero", &Tensor::set_zero)
        .def("add", &Tensor::add)
        .def("sub", &Tensor::sub)
        .def("mult", &Tensor::mult)
        .def("div", &Tensor::div)
        .def("scalar_mult", &Tensor::scalar_mult)
        .def("__repr__", [](const Tensor& t) {
            std::ostringstream oss;
            t.write(oss);
            return oss.str();
        });

    // INode
    py::class_<g::INode, std::shared_ptr<g::INode>>(m, "INode")
        .def("value", &g::INode::value)
        .def_static(
            "factory",
            [](const std::string& classname, const std::string& nodename) {
                return g::INode::factory(classname, nodename);
            },
            py::arg("classname"), py::arg("nodename") = "");
    m.def(
        "set_dep",
        [](const std::shared_ptr<g::INode>& node, const std::vector<std::shared_ptr<g::INode>>& deps) {
            for (auto& prev_node : deps)
            {
                node->add_prev(prev_node);
                prev_node->add_next(node);
            }
        },
        py::arg("node"), py::arg("deps"));

    // DataNode
    py::class_<g::DataNode, g::INode, std::shared_ptr<g::DataNode>>(m, "DataNode")
        .def(py::init<const std::string&>())
        .def("set_value", &g::DataNode::set_value)
        .def("value", &g::DataNode::value);

    // Model
    py::class_<g::Model>(m, "Model")
        .def(py::init<const std::vector<g::pnode_t>&, const std::vector<g::pnode_t>&>(), py::arg("inputs"),
             py::arg("outputs"))

        .def_static("load", &g::Model::load, py::arg("filename"))
        .def("save", &g::Model::save, py::arg("filename"))
        .def("input_nodes", &g::Model::input_nodes)
        .def("output_nodes", &g::Model::output_nodes)
        .def("nodes", &g::Model::nodes)
        .def("param_nodes", &g::Model::param_nodes)
        .def("set_param_nodes", &g::Model::set_param_nodes)
        .def("compute", &g::Model::compute, py::arg("input_values"));

    // OptimizationWorker
    py::class_<g::OptimizationWorker>(m, "OptimizationWorker")
        .def(py::init<const g::Model&, g::LossType>(), py::arg("model"), py::arg("loss_type"))

        .def("set_optimizer", &g::OptimizationWorker::set_optimizer, py::arg("optimizer"))
        .def("train", &g::OptimizationWorker::train, py::arg("inputs"), py::arg("gt"))
        .def("validate", &g::OptimizationWorker::validate, py::arg("inputs"), py::arg("gt"))
        .def("commit", &g::OptimizationWorker::commit);

    // LossType
    py::enum_<g::LossType>(m, "LossType").value("MSE", g::LossType::MSE);

    // Optimizer
    py::class_<g::IOptimizer, std::shared_ptr<g::IOptimizer>>(m, "IOptimizer")
        .def("set_param_nodes", &g::IOptimizer::set_param_nodes);

    // SGDOptimizer
    py::class_<g::SGDOptimizer, g::IOptimizer, std::shared_ptr<g::SGDOptimizer>>(m, "SGDOptimizer")
        .def(py::init<double>(), py::arg("learning_rate"));

    // IDataGenerator
    py::class_<g::IDataGenerator, std::shared_ptr<g::IDataGenerator>>(m, "IDataGenerator")
        .def("next_input", &g::IDataGenerator::next_input)
        .def("next_gt", &g::IDataGenerator::next_gt)
        .def("is_epoch_end", &g::IDataGenerator::is_epoch_end)
        .def("next_epoch", &g::IDataGenerator::next_epoch, py::arg("shuffle"));

    // SimpleDataGenerator
    py::class_<g::SimpleDataGenerator, g::IDataGenerator, std::shared_ptr<g::SimpleDataGenerator>>(
        m, "SimpleDataGenerator")
        .def(py::init<const std::vector<g::tvec_t>&, const std::vector<g::tvec_t>&, int, int>(), py::arg("inputs"),
             py::arg("outputs"), py::arg("batch_size") = 1, py::arg("seed") = 0);
}