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
    _value.serialize(js);
}
Tensor DataNode::notself_derivative(const INode* arg)
{
    log().debug("Gradient in DataNode compute");
    return Tensor(arg->get_shape(), 0.0);
}

std::string DataNode::classname() const
{
    return "DataNode";
}

Shape DataNode::get_shape() const
{
    return _value.get_shape();
}