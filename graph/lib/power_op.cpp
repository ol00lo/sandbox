#include "power_op.hpp"
#include "power_nodes.hpp"
using namespace g;

std::shared_ptr<IFunctionalNode> op::sqr(std::shared_ptr<INode> a)
{
    std::shared_ptr<IFunctionalNode> b(new SqrNode);
    b->add_prev(a);
    a->add_next(b);
    return b;
}