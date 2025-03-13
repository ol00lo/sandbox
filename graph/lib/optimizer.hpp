#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "training_graph.hpp"

namespace g
{
struct IOptimizer
{
    void set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& params);
    virtual void apply(const std::vector<Tensor>& gradients) = 0;
    virtual ~IOptimizer(){}

protected:
    std::vector<std::shared_ptr<DataNode>> _params;
};

struct SGDOptimizer : public IOptimizer
{
public:
    SGDOptimizer(double lr) : IOptimizer(), _learning_rate(lr) {}
    void apply(const std::vector<Tensor>& gradients) override;

private:
    double _learning_rate;
};
} // namespace g
#endif