#include "optimization_worker.hpp"

using namespace g;

void OptimizationWorker::set_optimizer(const std::shared_ptr<IOptimizer> optimizer)
{
    optimizer_ = optimizer;
    optimizer_->set_param_nodes(graph_.param_nodes());
}

double OptimizationWorker::train(const std::vector<Tensor>& inputs, const std::vector<Tensor>& gt)
{
    double loss = compute_loss(inputs, gt);
    auto grads = graph_.gradients();
    optimizer_->apply(grads);
    log().info("Train loss: {}", loss);
    return loss;
}

double OptimizationWorker::validate(const std::vector<Tensor>& inputs, const std::vector<Tensor>& gt)
{
    double loss = compute_loss(inputs, gt);
    log().info("Val loss:   {}", loss);
    return loss;
}

void OptimizationWorker::commit()
{
    graph_.copy_to_target();
}

double OptimizationWorker::compute_loss(const std::vector<Tensor>& inputs, const std::vector<Tensor>& gt)
{
    double res = 0;
    std::vector<Tensor> local_inputs = inputs;
    for (const auto& x : gt)
    {
        local_inputs.push_back(x);
    }
    auto out = graph_.compute(local_inputs);
    if (out.size() != 1)
    {
        _THROW_NOT_IMP_
    }
    res += out[0][0];
    return res;
}