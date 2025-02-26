#include "training_graph.hpp"

using namespace g;

TrainingGraph::TrainingGraph(Model target_model, std::string loss_type) : _target_model(target_model)
{
    Model copy_model = Model::deserialize(target_model.serialize());
    _input_nodes = copy_model.get_input_nodes();
    _param_nodes = copy_model.get_param_nodes();
    _inter_nodes = copy_model.get_inter_nodes();
    auto out_vector = copy_model.get_output_nodes();
    if (out_vector.size() != 1)
    {
        throw std::runtime_error("Not implemented");
    }
    auto out = out_vector[0];

    if (loss_type == "MSE")
    {
        for (int i = 0; i < out_vector.size(); i++)
        {
            std::shared_ptr<DataNode> gr_t = std::make_shared<DataNode>("");
            _input_nodes.push_back(gr_t);
            INode::PNode minus = INode::factory("MinusNode", "");
            g::set_dep(minus, {gr_t, out});
            _inter_nodes.push_back(minus);
            INode::PNode loss = INode::factory("SqrNode", "");
            g::set_dep(loss, {minus});
            _inter_nodes.push_back(loss);
            _output_nodes.push_back(loss);
        }
    }
}

void TrainingGraph::copy_to_target()
{
    _target_model.set_param(_param_nodes);
}

std::vector<Tensor> TrainingGraph::get_gradients() const
{
    std::vector<Tensor> res;
    for (int i = 0; i < _param_nodes.size(); i++)
        res.push_back(_output_nodes[0]->get_derivative(_param_nodes[i]));
    return res;
}