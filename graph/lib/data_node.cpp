#include "data_node.hpp"

using namespace g;

double DataNode::get_value()
{
    return _value;
}

void DataNode::set_value(double val)
{
    log().info("Data node value changed: from {} to {}", _value, val);
    _value = val;
    clear_forward_cache();
}

void DataNode::serialize_spec(nlohmann::json& js) const
{
    js["value"] = _value;
}
double DataNode::notself_derivative(const INode* arg)
{
    log().debug("Gradient in DataNode compute");
    return 0.0;
}

std::string DataNode::classname() const
{
    return "DataNode";
}