#include "graph.hpp"
#include <iostream>
void INode::add_prev(const std::shared_ptr<INode>& a)
{
    _prev_nodes.push_back(a);
}
void INode::add_next(const std::shared_ptr<INode>& a)
{
    _next_nodes.push_back(a);
}
void INode::clear_forward_cache()
{
    clear_cache();
    for (auto& n : _next_nodes)
    {
        n->clear_forward_cache();
    }
}

void INode::clear_backward_cache()
{
    clear_cache();
    for (auto& n : _prev_nodes)
    {
        n->clear_backward_cache();
    }
}

double InputNode::get_value()
{
    return _value;
}

void InputNode::set_value(double val)
{
    _value = val;
    clear_forward_cache();
}