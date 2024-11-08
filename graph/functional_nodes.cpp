#include "functional_nodes.hpp"
#include <iostream>

void IFunctionalNode::add_value_callback(callback_t cb)
{
    _value_callbacks.push_back(cb);
}

double IFunctionalNode::get_value()
{
    if (!_has_value)
    {
        std::cout << "Computing value: IFunctionalNode\n";
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
    std::cout << "Cache cleared: IFunctionalNode\n";
}

void IFunctionalNode::before_value_compute()
{
    for (auto& c : _value_callbacks)
    {
        c(this);
    }
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

double SqrNode::compute_value()
{
    double value = _prev_nodes[0]->get_value();
    return value * value;
}