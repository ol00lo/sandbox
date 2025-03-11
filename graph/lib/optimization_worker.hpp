#ifndef OPTIMIZATION_WORKER_H
#define OPTIMIZATION_WORKER_H

#include "optimizer.hpp"

namespace g
{
class OptimizationWorker
{
public:
    OptimizationWorker(const Model& model, LossType loss_type) : _graph(model, loss_type) {}
    void set_optimizer(std::shared_ptr<IOptimizer> optimizer);
    double train(const std::vector<Tensor>& inputs, const std::vector<Tensor>& gt);
    double validate(const std::vector<Tensor>& inputs, const std::vector<Tensor>& gt);
    void commit();

private:
    TrainingGraph _graph;
    std::shared_ptr<IOptimizer> _optimizer;
    double compute_loss(const std::vector<Tensor>& inputs, const std::vector<Tensor>& gt);
};
} // namespace g

#endif