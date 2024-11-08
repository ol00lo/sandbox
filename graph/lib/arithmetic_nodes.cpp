#include "arithmetic_nodes.hpp"
using namespace g;

MultNode::MultNode(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    add_prev(a1);
    add_prev(a2);
}

double MultNode::compute_value()
{
    return _prev_nodes[0]->get_value() * _prev_nodes[1]->get_value();
}

PlusNode::PlusNode(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    add_prev(a1);
    add_prev(a2);
}

double PlusNode::compute_value()
{
    return _prev_nodes[0]->get_value() + _prev_nodes[1]->get_value();
}

MinusNode::MinusNode(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    add_prev(a1);
    add_prev(a2);
}

double MinusNode::compute_value()
{
    return _prev_nodes[0]->get_value() - _prev_nodes[1]->get_value();
}