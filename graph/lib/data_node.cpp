#include "data_node.hpp"

using namespace g;

Tensor DataNode::get_value()
{
    return _value;
}

void DataNode::set_value(Tensor val)
{
    _value = val;
    log().info("Data node \"{}\" value changed", nodename());
    clear_forward_cache();
}

void DataNode::serialize_spec(nlohmann::json& js) const
{
    js["value"] = std::vector<double>(_value);
}
Tensor DataNode::notself_derivative(const INode* arg)
{
    log().debug("Gradient in DataNode compute");
    return Tensor({0});
}

std::string DataNode::classname() const
{
    return "DataNode";
}