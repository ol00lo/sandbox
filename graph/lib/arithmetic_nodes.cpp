#include "arithmetic_nodes.hpp"
using namespace g;

double MultNode::compute_value()
{
    auto res = _prev_nodes[0]->get_value() * _prev_nodes[1]->get_value();
    log().debug("MultNode compute: {:.2f} * {:.2f} = {:.2f}", _prev_nodes[0]->get_value(), _prev_nodes[1]->get_value(),
                res);
    return res;
}

double PlusNode::compute_value()
{
    auto res = _prev_nodes[0]->get_value() + _prev_nodes[1]->get_value();
    log().debug("PlusNode compute: {:.2f} + {:.2f} = {:.2f}", _prev_nodes[0]->get_value(), _prev_nodes[1]->get_value(),
                res);
    return res;
}

double MinusNode::compute_value()
{
    auto res = _prev_nodes[0]->get_value() - _prev_nodes[1]->get_value();
    log().debug("MinusNode compute: {:.2f} - {:.2f} = {:.2f}", _prev_nodes[0]->get_value(), _prev_nodes[1]->get_value(),
                res);
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
std::vector<double> MultNode::get_gradient()
{
    log().debug("Gradient in MultNode compute");
    double val1 = _prev_nodes[0]->get_value();
    double val2 = _prev_nodes[1]->get_value();
    std::vector<double> res = {val2, val1};
    return res;
}

std::vector<double> PlusNode::get_gradient()
{
    log().debug("Gradient in PlusNode compute");
    std::vector<double> res = {1.0, 1.0};
    return res;
}

std::vector<double> MinusNode::get_gradient()
{
    log().debug("Gradient in MinusNode compute");
    std::vector<double> res = {1.0, -1.0};
    return res;
}

std::shared_ptr<IFunctionalNode> op::mult(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    std::shared_ptr<IFunctionalNode> a(new MultNode);
    add_dependencies(a, {a1, a2});
    return a;
}

std::shared_ptr<IFunctionalNode> op::plus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    std::shared_ptr<IFunctionalNode> a(new PlusNode);
    add_dependencies(a, {a1, a2});
    return a;
}

std::shared_ptr<IFunctionalNode> op::minus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    std::shared_ptr<IFunctionalNode> a(new MinusNode);
    add_dependencies(a, {a1, a2});
    return a;
}