#include "optimizer.hpp"

using namespace g;

void IOptimizer::set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& params)
{
    params_ = params;
}

void SGDOptimizer::apply(const std::vector<Tensor>& gradients)
{
    for (size_t i = 0; i < params_.size(); i++)
    {
        Tensor delt = g::scalar_mult(learning_rate_, gradients[i]);
        Tensor old_value = params_[i]->value();
        Tensor res = g::sub(old_value, delt);
        params_[i]->set_value(res);
    }
}