#include "training_graph.hpp"
#include "arithmetic_nodes.hpp"
#include "power_nodes.hpp"

using namespace g;

TrainingGraph::TrainingGraph(const Model& target_model, LossType loss_type) : _target_model(target_model), Model()
{
    nlohmann::json j;
    nlohmann::adl_serializer<g::Model>::to_json(j, target_model);
    nlohmann::adl_serializer<g::Model>::from_json(j, *this);

    if (_output_nodes.size() != 1)
    {
        _THROW_NOT_IMP_
    }
    auto out_vector = std::move(_output_nodes);
    auto out = out_vector[0];

    switch (loss_type)
    {
    case g::LossType::MSE:
        for (int i = 0; i < out_vector.size(); i++)
        {
            std::shared_ptr<DataNode> gr_t = std::make_shared<DataNode>("");
            _input_nodes.push_back(gr_t);
            INode::ptr_t minus = std::make_shared<MinusNode>("");
            g::set_dep(minus, {gr_t, out});
            _inter_nodes.push_back(minus);
            INode::ptr_t loss = std::make_shared<SqrNode>("");
            g::set_dep(loss, {minus});
            _inter_nodes.push_back(loss);
            _output_nodes.push_back(loss);
        }
        break;
    default:
        _THROW_NOT_IMP_
        break;
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