#include "power_nodes.hpp"
using namespace g;

Tensor SqrNode::compute_value()
{
    Tensor value = prev_nodes_[0]->value();
    Tensor res = mult(value, value);
    log().debug("SqrNode  \"{}\" compute", nodename());
    return res;
}

std::string SqrNode::classname() const
{
    return "SqrNode";
}

std::vector<Tensor> SqrNode::gradient()
{
    Tensor val = scalar_mult(2.0, prev_nodes_[0]->value());
    log().debug("Gradient in SqrNode compute");
    std::vector<Tensor> res = {val};
    return res;
}