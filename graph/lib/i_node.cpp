#include "i_node.hpp"
using namespace g;

void INode::add_prev(const std::shared_ptr<INode> a)
{
    _prev_nodes.push_back(a);
}
void INode::add_next(const std::shared_ptr<INode> a)
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

INode::~INode()
{
    clear_forward_cache();
    clear_backward_cache();
}