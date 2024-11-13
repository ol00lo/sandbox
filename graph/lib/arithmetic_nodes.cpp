#include "arithmetic_nodes.hpp"
using namespace g;

double MultNode::compute_value()
{
    return _prev_nodes[0]->get_value() * _prev_nodes[1]->get_value();
}

double PlusNode::compute_value()
{
    return _prev_nodes[0]->get_value() + _prev_nodes[1]->get_value();
}

double MinusNode::compute_value()
{
    return _prev_nodes[0]->get_value() - _prev_nodes[1]->get_value();
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