#ifndef POWER_NODES_HPP
#define POWER_NODES_HPP
#include "i_functions_node.hpp"

namespace g
{
class SqrNode : public IFunctionalNode
{
protected:
    void log_cache() override;
    double compute_value() override;
};

namespace op
{
std::shared_ptr<IFunctionalNode> sqr(std::shared_ptr<INode> a);
} // namespace op
} // namespace g
#endif