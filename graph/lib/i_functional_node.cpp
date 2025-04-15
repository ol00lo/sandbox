#include "i_functional_node.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace g;

namespace
{
std::string pad_string(const std::string& str, size_t width = 10)
{
    std::ostringstream oss;
    oss << std::setw(width) << std::left << str;
    return oss.str();
}
} // namespace

void IFunctionalNode::add_value_callback(callback_t cb)
{
    value_callbacks_.push_back(cb);
}
void IFunctionalNode::add_gradient_callback(callback_t cb)
{
    gradient_callbacks_.push_back(cb);
}

Tensor IFunctionalNode::value()
{
    if (!has_value_)
    {
        before_value_compute();
        value_ = compute_value();
        log().trace("Value in {} {} compute.", pad_string(classname()),
                    pad_string(std::string("\"") + nodename() + std::string("\""), 6));
        has_value_ = true;
    }
    return value_;
}
Shape IFunctionalNode::output_shape() const
{
    return prev_nodes_[0]->output_shape();
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

void IFunctionalNode::log_cache() const
{
    log().debug(classname() + " cleaned");
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
        log().trace("Gradient in {} {} compute.", pad_string(classname()),
                    pad_string(std::string("\"") + nodename() + std::string("\""), 6));
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
        gradient_ = gradient();
        has_gradient_ = true;
    }
    for (int i = 0; i < prev_nodes_.size(); i++)
    {
        auto pr_der = prev_nodes_[i]->derivative(arg);
        if (gradient_[i].shape() == pr_der.shape())
            res.add(mult(gradient_[i], pr_der));
        else
            _THROW_NOT_IMP_
    }
    return res;
}