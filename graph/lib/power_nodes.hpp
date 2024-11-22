#ifndef POWER_NODES_HPP
#define POWER_NODES_HPP
#include "i_functional_node.hpp"

namespace g
{
class SqrNode : public IFunctionalNode
{
protected:
    double compute_value() override;
    std::string classname() const override;
    std::vector<double> get_gradient() override;
};

namespace op
{
std::shared_ptr<IFunctionalNode> sqr(std::shared_ptr<INode> a);
} // namespace op
} // namespace g
#endif