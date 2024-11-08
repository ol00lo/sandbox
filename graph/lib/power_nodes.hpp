#ifndef POWER_NODES_HPP
#define POWER_NODES_HPP
#include "i_functions_node.hpp"
namespace g
{
class SqrNode : public IFunctionalNode
{
protected:
    double compute_value() override;
};
} // namespace g
#endif