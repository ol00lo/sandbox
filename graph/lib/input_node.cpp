#include "input_node.hpp"

using namespace g;

double InputNode::get_value()
{
    return _value;
}

void InputNode::set_value(double val)
{
    log()->info("from {} to {}", _value, val);
    _value = val;
    clear_forward_cache();
}