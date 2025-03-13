#include "optimizer.hpp"

using namespace g;

void IOptimizer::set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& params)
{
    _params = params;
}

void SGDOptimizer::apply(const std::vector<Tensor>& gradients)
{
    for (size_t i = 0; i < _params.size(); i++)
    {

        Tensor delt = g::scalar_mult(_learning_rate, gradients[i]);
        Tensor old_value = _params[i]->get_value();
        Tensor res = g::sub(old_value, delt);
        _params[i]->set_value(res);
    }
}