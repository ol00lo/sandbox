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
