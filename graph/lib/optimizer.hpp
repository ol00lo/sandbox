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
    SGDOptimizer(double lr) : _learning_rate(lr)
    {
    }
    void set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& params) override;
    void apply(const std::vector<Tensor>& gradients) override;

private:
    std::vector<std::shared_ptr<DataNode>> _params;
    double _learning_rate;
};

class OptimizationWorker
{
public:
    OptimizationWorker(Model model, std::string loss_type) : _graph(model, loss_type){}
    void set_optimizer(std::shared_ptr<IOptimizer> optimizer);
    double train(std::vector<Tensor> inputs, std::vector<Tensor> gt, int batch_size = 1);
    double validate(std::vector<Tensor> inputs, std::vector<Tensor> gt, int batch_size = 1);
    void commit();

private:
    TrainingGraph _graph;
    std::shared_ptr<IOptimizer> _optimizer;
};
} // namespace g
#endif