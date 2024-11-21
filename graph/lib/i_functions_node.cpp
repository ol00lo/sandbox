#include "i_functions_node.hpp"
#include <iostream>

using namespace g;

void IFunctionalNode::add_value_callback(callback_t cb)
{
    _value_callbacks.push_back(cb);
}

double IFunctionalNode::get_value()
{
    if (!_has_value)
    {
        before_value_compute();
        _value = compute_value();
        _has_value = true;
    }
    return _value;
}

void IFunctionalNode::clear_cache()
{
    log_cache();
    _derivative_cache.clear();
    log().debug("derivative cache cleaned");
    _has_value = false;
    _value = 0;
}

void IFunctionalNode::before_value_compute()
{
    for (auto& c : _value_callbacks)
    {
        c(this);
    }
}

double IFunctionalNode::notself_derivative(const INode* arg)
{
    auto x = _derivative_cache.find(arg);
    if (x == _derivative_cache.end() || _has_value == false)
    {
        double res = compute_notself_derivative(arg);
        _derivative_cache.insert({arg, res});
        return res;
    }
    else
    {
        return x->second;
    }
}

double IFunctionalNode::compute_notself_derivative(const INode* arg)
{
    double res = 0;
    for (int i = 0; i < _prev_nodes.size(); i++)
    {
        res += gradient()[i] * _prev_nodes[i]->derivative(arg);
    }
    return res;
}

void g::add_dependencies(std::shared_ptr<IFunctionalNode> node,
                                       std::initializer_list<std::shared_ptr<INode>> prevs)
{
    for (auto p : prevs)
    {
        node->add_prev(p);
        p->add_next(node);
    }
}
