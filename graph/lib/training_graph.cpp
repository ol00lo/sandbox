#include "training_graph.hpp"
#include "arithmetic_nodes.hpp"
#include "power_nodes.hpp"

using namespace g;

TrainingGraph::TrainingGraph(const Model& target_model, LossType loss_type) : target_model_(target_model), Model()
{
    nlohmann::json j;
    nlohmann::adl_serializer<g::Model>::to_json(j, target_model);
    nlohmann::adl_serializer<g::Model>::from_json(j, *this);

    if (output_nodes_.size() != 1)
    {
        _THROW_NOT_IMP_
    }
    auto out_vector = std::move(output_nodes_);
    auto out = out_vector[0];

    switch (loss_type)
    {
    case g::LossType::MSE:
        for (int i = 0; i < out_vector.size(); i++)
        {
            std::shared_ptr<DataNode> gr_t = std::make_shared<DataNode>("");
            input_nodes_.push_back(gr_t);
            INode::ptr_t minus = std::make_shared<MinusNode>("");
            g::set_dep(minus, {gr_t, out});
            inter_nodes_.push_back(minus);
            INode::ptr_t loss = std::make_shared<SqrNode>("");
            g::set_dep(loss, {minus});
            inter_nodes_.push_back(loss);
            output_nodes_.push_back(loss);
        }
        break;
    default:
        _THROW_NOT_IMP_
        break;
    }
}

void TrainingGraph::copy_to_target()
{
    target_model_.set_param(param_nodes_);
}

std::vector<Tensor> TrainingGraph::get_gradients() const
{
    std::vector<Tensor> res;
    for (int i = 0; i < param_nodes_.size(); i++)
        res.push_back(output_nodes_[0]->get_derivative(param_nodes_[i]));
    return res;
}