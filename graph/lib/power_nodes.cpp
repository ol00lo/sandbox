#include "power_nodes.hpp"
using namespace g;

Tensor SqrNode::compute_value()
{
    Tensor value = _prev_nodes[0]->get_value();
    Tensor res = mult(value, value);
    log().debug("SqrNode  \"{}\" compute", nodename());
    return res;
}

std::string SqrNode::classname() const
{
    return "SqrNode";
}

std::vector<Tensor> SqrNode::get_gradient()
{
    Tensor val = scalar_mult(2.0, _prev_nodes[0]->get_value());
    log().debug("Gradient in SqrNode compute");
    std::vector<Tensor> res = {val};
    return res;
}