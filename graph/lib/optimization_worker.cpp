#include "optimization_worker.hpp"

using namespace g;

void OptimizationWorker::set_optimizer(const std::shared_ptr<IOptimizer> optimizer)
{
    _optimizer = optimizer;
    _optimizer->set_param_nodes(_graph.get_param_nodes());
}

double OptimizationWorker::train(const std::vector<Tensor>& inputs, const std::vector<Tensor>& gt)
{
    double res = compute_loss(inputs, gt);
    auto grads = _graph.get_gradients();
    _optimizer->apply(grads);
    return res;
}

double OptimizationWorker::validate(const std::vector<Tensor>& inputs, const std::vector<Tensor>& gt)
{
    return compute_loss(inputs, gt);
}

void OptimizationWorker::commit()
{
    _graph.copy_to_target();
}

double OptimizationWorker::compute_loss(const std::vector<Tensor>& inputs, const std::vector<Tensor>& gt)
{
    double res = 0;
    std::vector<Tensor> local_inputs = inputs;
    for (const auto& x : gt)
    {
        local_inputs.push_back(x);
    }
    auto out = _graph.compute(local_inputs);
    if (out.size() != 1)
    {
        _THROW_NOT_IMP_
    }
    res += out[0][0];
    return res;
}