#include "arithmetic_nodes.hpp"
using namespace g;

double MultNode::compute_value()
{
    auto res = _prev_nodes[0]->get_value() * _prev_nodes[1]->get_value();
    log()->info("{:.2f} * {:.2f} = {:.2f}", _prev_nodes[0]->get_value(), _prev_nodes[1]->get_value(), res);
    return res;
}

double PlusNode::compute_value()
{
    auto res = _prev_nodes[0]->get_value() + _prev_nodes[1]->get_value();
    log()->info("{:.2f} + {:.2f} = {:.2f}", _prev_nodes[0]->get_value(), _prev_nodes[1]->get_value(), res);
    return res;
}

double MinusNode::compute_value()
{
    auto res = _prev_nodes[0]->get_value() - _prev_nodes[1]->get_value();
    log()->info("{:.2f} - {:.2f} = {:.2f}", _prev_nodes[0]->get_value(), _prev_nodes[1]->get_value(), res);
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