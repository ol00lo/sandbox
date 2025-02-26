#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include "training_graph.hpp"

namespace g
{
struct IOptimizer
{
    virtual void set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& params) = 0;
    virtual void apply(const std::vector<Tensor>& gradients) = 0;
    virtual ~IOptimizer(){}
};

struct SGDOptimizer : public IOptimizer
{
public:
    SGDOptimizer(Tensor lr) : _learning_rate(lr){}
    void set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& params) override;
    void apply(const std::vector<Tensor>& gradients) override;

private:
    std::vector<std::shared_ptr<DataNode>> _params;
    Tensor _learning_rate;
};

class OptimizationWorker
{
public:
    OptimizationWorker(Model model, std::string loss_type) : _graph(model, loss_type){}
    void set_optimizer(std::shared_ptr<IOptimizer> optimizer);
    Tensor train(std::vector<Tensor> inputs, std::vector<Tensor> gt);
    Tensor validate(std::vector<Tensor> inputs, std::vector<Tensor> gt);
    void commit();

private:
    TrainingGraph _graph;
    std::shared_ptr<IOptimizer> _optimizer;
};
} // namespace g
#endif