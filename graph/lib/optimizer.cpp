#include "optimizer.hpp"

using namespace g;

void SGDOptimizer::set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& params) 
{
    _params = params;
}

void SGDOptimizer::apply(const std::vector<Tensor>& gradients) 
{
    for (size_t i = 0; i < _params.size(); i++)
    {
        Tensor delt = g::mult(gradients[i], _learning_rate);
        Tensor old_value = _params[i]->get_value();
        Tensor res = g::sub(old_value, delt);
        _params[i]->set_value(res);
    }
}

void OptimizationWorker::set_optimizer(std::shared_ptr<IOptimizer> optimizer)
{
    _optimizer = optimizer;
}

Tensor OptimizationWorker::train(std::vector<Tensor> inputs, std::vector<Tensor> gt)
{
    if (inputs.size() != gt.size())
    {
        throw std::runtime_error("Inputs and ground truth must have the same size.");
    }
    _optimizer->set_param_nodes(_graph.get_param_nodes());

    for (int i = 0; i < gt.size(); i++)
    {
        inputs.push_back(gt[i]);
    }
    auto out = _graph.compute(inputs);

    Tensor res({0});
	for (const auto &x: out)
	{
        res.add(x);
	}
    res.div(Tensor({double(out.size())}));

    auto grads = _graph.get_gradients();
    _optimizer->apply(grads);
    return res;
}

Tensor OptimizationWorker::validate(std::vector<Tensor> inputs, std::vector<Tensor> gt)
{
    _optimizer->set_param_nodes(_graph.get_param_nodes());
    for (int i = 0; i < gt.size(); i++)
    {
        inputs.push_back(gt[i]);
    }
    auto out = _graph.compute(inputs);
    Tensor res({0});
    for (const auto &x: out)
    {
        res.add(x);
    }
    Tensor N();
    res.div(Tensor({double(out.size())}));
    return res;
}

void OptimizationWorker::commit()
{
    _graph.copy_to_target();
}