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
    js["value"] = _value;
}

void DataNode::deserialize_spec(const nlohmann::json& node_json, std::string copy_word)
{
    std::string nname = node_json["nodename"].get<std::string>() + copy_word;
    std::vector<double> x = node_json["value"]["value"].get<std::vector<double>>();
    Shape shape = node_json["value"]["shape"].get<Arr4>();
    Tensor val(shape, x);
    set_value(val);    
}
Tensor DataNode::notself_derivative(const INode* arg)
{
    log().debug("Gradient in DataNode compute");
    return Tensor(arg->output_shape(), 0.0);
}

std::string DataNode::classname() const
{
    return "DataNode";
}

Shape DataNode::output_shape() const
{
    return _value.get_shape();
}