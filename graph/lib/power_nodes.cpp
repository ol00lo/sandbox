#include "power_nodes.hpp"
using namespace g;

void add_dependencies(std::shared_ptr<INode> a, std::shared_ptr<IFunctionalNode> b)
{
    b->add_prev(a);
    a->add_next(b);
}

double SqrNode::compute_value()
{
    double value = _prev_nodes[0]->get_value();
    double res = value * value;
    log().debug("SqrNode compute: {:.2f}^2 = {:.2f}", value, res);
    return res;
}
void SqrNode::log_cache()
{
    log().debug("SqrNode cleaned");
}
std::shared_ptr<IFunctionalNode> op::sqr(std::shared_ptr<INode> a)
{
    std::shared_ptr<IFunctionalNode> b(new SqrNode());
    add_dependencies(b, {a});
    return b;
}