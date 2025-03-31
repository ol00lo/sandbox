#include "i_functional_node.hpp"
#include <iostream>

using namespace g;

void IFunctionalNode::add_value_callback(callback_t cb)
{
    value_callbacks_.push_back(cb);
}
void IFunctionalNode::add_gradient_callback(callback_t cb)
{
    gradient_callbacks_.push_back(cb);
}

Tensor IFunctionalNode::get_value()
{
    if (!has_value_)
    {
        before_value_compute();
        value_ = compute_value();
        has_value_ = true;
    }
    return value_;
}

void IFunctionalNode::clear_cache()
{
    log_cache();
    has_gradient_ = false;
    derivative_cache_.clear();
    log().debug("derivative cache cleaned");
    has_value_ = false;
    value_.set_zero();
}

void IFunctionalNode::before_value_compute()
{
    for (auto& c : value_callbacks_)
    {
        c(this);
    }
}
void IFunctionalNode::before_gradient_compute()
{
    for (auto& c : gradient_callbacks_)
    {
        c(this);
    }
}

Tensor IFunctionalNode::notself_derivative(const INode* arg)
{
    auto x = derivative_cache_.find(arg);
    if (x == derivative_cache_.end())
    {
        Tensor res = compute_notself_derivative(arg);
        derivative_cache_.insert({arg, res});
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
    if (!has_gradient_)
    {
        before_gradient_compute();
        gradient_ = get_gradient();
        has_gradient_ = true;
    }
    for (int i = 0; i < prev_nodes_.size(); i++)
    {
        auto pr_der = prev_nodes_[i]->get_derivative(arg);
        if (gradient_[i].get_shape() == pr_der.get_shape())
            res.add(mult(gradient_[i], pr_der));
        else
            _THROW_NOT_IMP_
    }
    return res;
}
Shape IFunctionalNode::output_shape() const
{
    return prev_nodes_[0]->output_shape();
}