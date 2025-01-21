#include "arithmetic_nodes.hpp"
using namespace g;

Tensor MultNode::compute_value()
{
    Tensor res = mult(_prev_nodes[0]->get_value(), _prev_nodes[1]->get_value());
    log().debug("MultNode  \"{}\" compute}", nodename());
    return res;
}

Tensor PlusNode::compute_value()
{
    Tensor res = add(_prev_nodes[0]->get_value(), _prev_nodes[1]->get_value());
    log().debug("PlusNode  \"{}\" compute}", nodename());
    return res;
}

Tensor MinusNode::compute_value()
{
    Tensor res = sub(_prev_nodes[0]->get_value(), _prev_nodes[1]->get_value());
    log().debug("MinusNode  \"{}\" compute}", nodename());
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

std::vector<Tensor> MultNode::get_gradient()
{
    log().debug("Gradient in MultNode compute");
    Tensor val1 = _prev_nodes[0]->get_value();
    Tensor val2 = _prev_nodes[1]->get_value();
    std::vector<Tensor> res = {val2, val1};
    return res;
}

std::vector<Tensor> PlusNode::get_gradient()
{
    log().debug("Gradient in PlusNode compute");
    std::vector<Tensor> res = {Tensor({1.0}), Tensor({1.0})};
    return res;
}

std::vector<Tensor> MinusNode::get_gradient()
{
    log().debug("Gradient in MinusNode compute");
    std::vector<Tensor> res = {Tensor({1.0}), Tensor({-1.0})};
    return res;
}