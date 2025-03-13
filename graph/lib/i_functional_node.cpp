#include "i_functional_node.hpp"
#include <iostream>

using namespace g;

void IFunctionalNode::add_value_callback(callback_t cb)
{
    _value_callbacks.push_back(cb);
}
void IFunctionalNode::add_gradient_callback(callback_t cb)
{
    _gradient_callbacks.push_back(cb);
}

Tensor IFunctionalNode::get_value()
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
    _has_gradient = false;
    _derivative_cache.clear();
    log().debug("derivative cache cleaned");
    _has_value = false;
    _value.set_zero();
}

void IFunctionalNode::before_value_compute()
{
    for (auto& c : _value_callbacks)
    {
        c(this);
    }
}
void IFunctionalNode::before_gradient_compute()
{
    for (auto& c : _gradient_callbacks)
    {
        c(this);
    }
}

Tensor IFunctionalNode::notself_derivative(const INode* arg)
{
    auto x = _derivative_cache.find(arg);
    if (x == _derivative_cache.end())
    {
        Tensor res = compute_notself_derivative(arg);
        _derivative_cache.insert({arg, res});
        return res;
    }
    else
    {
        return x->second;
    }
}

Tensor IFunctionalNode::compute_notself_derivative(const INode* arg)
{
    Tensor res(arg->output_shape(), 0.0);
    if (!_has_gradient)
    {
        before_gradient_compute();
        _gradient = get_gradient();
        _has_gradient = true;
    }
    for (int i = 0; i < _prev_nodes.size(); i++)
    {
        auto pr_der = _prev_nodes[i]->get_derivative(arg);
        if (_gradient[i].get_shape() == pr_der.get_shape())
            res.add(mult(_gradient[i], pr_der));
        else
            _THROW_NOT_IMP_
    }
    return res;
}
Shape IFunctionalNode::output_shape() const
{
    return _prev_nodes[0]->output_shape();
}