#include "trigonometric_nodes.hpp"
using namespace g;

Tensor SinNode::compute_value()
{
    Tensor pred = prev_nodes_[0]->value();
    Tensor res = sin(pred);
    return res;
}
std::string SinNode::classname() const
{
    return "SinNode";
}
std::vector<Tensor> SinNode::gradient()
{
    Tensor res = g::cos(prev_nodes_[0]->value());
    return {res};
}

Tensor CosNode::compute_value()
{
    Tensor pred = prev_nodes_[0]->value();
    Tensor res = cos(pred);
    return res;
}
std::string CosNode::classname() const
{
    return "CosNode";
}
std::vector<Tensor> CosNode::gradient()
{
    Tensor res = g::sin(prev_nodes_[0]->value());
    res.scalar_mult(-1.0);
    return {res};
}