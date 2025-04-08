#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "training_graph.hpp"

namespace g
{
struct IOptimizer
{
    void set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& params);
    virtual void apply(const std::vector<Tensor>& gradients) = 0;
    virtual ~IOptimizer(){};

protected:
    std::vector<std::shared_ptr<DataNode>> params_;
};

struct SGDOptimizer : public IOptimizer
{
public:
    SGDOptimizer(double lr) : IOptimizer(), learning_rate_(lr) {};
    void apply(const std::vector<Tensor>& gradients) override;

private:
    double learning_rate_;
};
} // namespace g
#endif