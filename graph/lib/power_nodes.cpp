#include "power_nodes.hpp"
using namespace g;

Tensor SqrNode::compute_value()
{
    Tensor value = prev_nodes_[0]->value();
    Tensor res = mult(value, value);
    return res;
}

std::string SqrNode::classname() const
{
    return "SqrNode";
}

std::vector<Tensor> SqrNode::gradient()
{
    Tensor val = scalar_mult(2.0, prev_nodes_[0]->value());
    std::vector<Tensor> res = {val};
    return res;
}