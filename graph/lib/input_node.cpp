#include "input_node.hpp"

using namespace g;

double InputNode::get_value()
{
    return _value;
}

void InputNode::set_value(double val)
{
    log().info("Input node value changed: from {} to {}", _value, val);
    _value = val;
    clear_forward_cache();
}
nlohmann::json InputNode::serialize() const
{
    return {{"classname", "InputNode"}, {"nodename", _layer_name}, {"value", _value}};
}
double InputNode::notself_derivative(const INode* arg)
{
    log().debug("Gradient in InputNode compute");
    return 0.0;
}

std::string InputNode::classname() const
{
    return "InputNode"; 
}
