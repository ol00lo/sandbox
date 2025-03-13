#include "trigonometric_nodes.hpp"
using namespace g;

Tensor SinNode::compute_value()
{
    Tensor pred = _prev_nodes[0]->get_value();
    Tensor res = sin(pred);
    log().debug("SinNode  \"{}\" compute", nodename());
    return res;
}
std::string SinNode::classname() const
{
    return "SinNode";
}
std::vector<Tensor> SinNode::get_gradient()
{
    Tensor res = g::cos(_prev_nodes[0]->get_value());
    log().debug("Gradient in SinNode compute");
    return {res};
}

Tensor CosNode::compute_value()
{
    Tensor pred = _prev_nodes[0]->get_value();
    Tensor res = cos(pred);
    log().debug("CosNode  \"{}\" compute", nodename());
    return res;
}
std::string CosNode::classname() const
{
    return "CosNode";
}
std::vector<Tensor> CosNode::get_gradient()
{
    Tensor res = g::sin(_prev_nodes[0]->get_value());
    res.scalar_mult(-1.0);
    log().debug("Gradient in CosNode compute");
    return {res};
}