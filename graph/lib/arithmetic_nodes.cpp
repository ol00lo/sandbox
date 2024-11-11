#include "arithmetic_nodes.hpp"
using namespace g;

void add_dependencies(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2, std::shared_ptr<IFunctionalNode> a)
{
    a->add_prev(a1);
    a->add_prev(a2);
    a1->add_next(a);
    a2->add_next(a);
}

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
    add_dependencies(a1, a2, a);
    return a;
}

std::shared_ptr<IFunctionalNode> op::plus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    std::shared_ptr<IFunctionalNode> a(new PlusNode);
    add_dependencies(a1, a2, a);
    return a;
}

std::shared_ptr<IFunctionalNode> op::minus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    std::shared_ptr<IFunctionalNode> a(new MinusNode);
    add_dependencies(a1, a2, a);
    return a;
}