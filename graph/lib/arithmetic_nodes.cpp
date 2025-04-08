#include "arithmetic_nodes.hpp"
using namespace g;

Tensor MultNode::compute_value()
{
    Tensor res = mult(prev_nodes_[0]->value(), prev_nodes_[1]->value());
    return res;
}

Tensor PlusNode::compute_value()
{
    Tensor res = add(prev_nodes_[0]->value(), prev_nodes_[1]->value());
    return res;
}

Tensor MinusNode::compute_value()
{
    Tensor res = sub(prev_nodes_[0]->value(), prev_nodes_[1]->value());
    return res;
}

std::string MultNode::classname() const
{
    return "MultNode";
}

std::string PlusNode::classname() const
{
    return "PlusNode";
}

std::string MinusNode::classname() const
{
    return "MinusNode";
}

std::vector<Tensor> MultNode::gradient()
{
    Tensor val1 = prev_nodes_[0]->value();
    Tensor val2 = prev_nodes_[1]->value();
    std::vector<Tensor> res = {val2, val1};
    return res;
}

std::vector<Tensor> PlusNode::gradient()
{
    std::vector<Tensor> res = {Tensor(prev_nodes_[0]->output_shape(), 1.0), Tensor(prev_nodes_[1]->output_shape(), 1.0)};
    return res;
}

std::vector<Tensor> MinusNode::gradient()
{
    std::vector<Tensor> res = {Tensor(prev_nodes_[0]->output_shape(), 1.0), Tensor(prev_nodes_[1]->output_shape(), -1.0)};
    return res;
}