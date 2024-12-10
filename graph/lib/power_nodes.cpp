#include "power_nodes.hpp"
using namespace g;

void SqrNode::set_prev_nodes(std::initializer_list<std::shared_ptr<INode>> args)
{
    for (auto& a : args)
    {
        _prev_nodes.push_back(a);
    }
}

double SqrNode::compute_value()
{
    double value = _prev_nodes[0]->get_value();
    double res = value * value;
    log().debug("SqrNode compute: {:.2f}^2 = {:.2f}", value, res);
    return res;
}

std::string SqrNode::classname() const
{
    return "SqrNode";
}

std::vector<double> SqrNode::get_gradient()
{
    double val = _prev_nodes[0]->get_value();
    log().debug("Gradient in SqrNode compute");
    std::vector<double> res = {2 * val};
    return res;
}