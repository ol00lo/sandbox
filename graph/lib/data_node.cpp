#include "data_node.hpp"

using namespace g;

Tensor DataNode::value()
{
    return value_;
}

void DataNode::set_value(Tensor val)
{
    value_ = val;
    log().debug("Data node \"{}\" value changed", nodename());
    clear_forward_cache();
}

void DataNode::serialize_spec(nlohmann::json& js) const
{
    js["value"] = value_;
}

void DataNode::deserialize_spec(const nlohmann::json& node_json)
{
    std::string nname = node_json["nodename"].get<std::string>();
    std::vector<double> x = node_json["value"]["value"].get<std::vector<double>>();
    Shape shape = node_json["value"]["shape"].get<Shape>();
    Tensor val(shape, x);
    set_value(val);
}
Tensor DataNode::notself_derivative(const INode* arg)
{
    log().debug("Gradient in DataNode \"{}\" compute", nodename());
    return Tensor(arg->output_shape(), 0.0);
}

std::string DataNode::classname() const
{
    return "DataNode";
}

Shape DataNode::output_shape() const
{
    return value_.shape();
}