#include "model.hpp"
#include "arithmetic_nodes.hpp"
#include "power_nodes.hpp"
#include "trigonometric_nodes.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace g;
namespace
{
INode::PNode find_node_by_name(std::unordered_map<std::string, INode::PNode> all_nodes, const std::string& name)
{
    auto it = all_nodes.find(name);
    if (it != all_nodes.end())
    {
        return it->second;
    }
    return nullptr;
}
} // namespace

Model::Model(std::vector<INode::PNode> inputs, std::vector<INode::PNode> outputs)
    : _input_nodes(inputs), _output_nodes(outputs)
{
    for (const auto& output : _output_nodes)
    {
        add_into_inter(output);
    }
}

std::vector<Tensor> Model::compute(const std::vector<Tensor>& input_values)
{
    if (input_values.size() > _input_nodes.size())
    {
        throw std::runtime_error("Size of input values does not match the number of input nodes.");
    }
    for (size_t i = 0; i < input_values.size(); ++i)
    {
        INode* get_node = _input_nodes[i].get();
        if (DataNode* dataNode = dynamic_cast<DataNode*>(get_node))
        {
            dataNode->set_value(input_values[i]);
        }
    }

    std::vector<Tensor> results;
    for (const auto& output : _output_nodes)
    {
        results.push_back(output->get_value());
    }
    return results;
}
std::vector<INode::PNode> Model::get_input_nodes() const
{
    return _input_nodes;
}
std::vector<INode::PNode> Model::get_inter_nodes() const
{
    return _inter_nodes;
}
std::vector<INode::PNode> Model::get_output_nodes() const
{
    return _output_nodes;
}
std::vector<std::shared_ptr<DataNode>> Model::get_param_nodes() const
{
    return _param_nodes;
}
bool Model::is_in_param(DataNode* node)
{
    for (const auto& param : _param_nodes)
    {
        if (param->nodename() == node->nodename())
        {
            return true;
        }
    }
    return false;
}
void Model::set_param(const std::vector<std::shared_ptr<DataNode>>& p)
{
    for (int i = 0; i < _param_nodes.size(); i++)
    {
        _param_nodes[i]->set_value(p[i]->get_value());
    }
}
void Model::add_into_inter(INode::PNode node)
{
    if (node->classname() == "DataNode")
    {
        auto fnd = std::find(_input_nodes.begin(), _input_nodes.end(), node);
        if (fnd != _input_nodes.end())
        {
            return;
        }
        else
        {
            auto dataNode = std::dynamic_pointer_cast<DataNode>(node);
            if (dataNode)
            {
                if (is_in_param(dataNode.get()))
                {
                    return;
                }
                _param_nodes.push_back(dataNode);
                _inter_nodes.push_back(dataNode);
            }
        }
    }
    else if (std::find(_inter_nodes.begin(), _inter_nodes.end(), node) == _inter_nodes.end())
    {
        _inter_nodes.push_back(node);
    }
    for (const auto& prev : node->get_prev())
    {
        add_into_inter(prev);
    }
}

void Model::save(const std::string& filename)
{

    std::ofstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open file for writing.");
    }
    file << serialize().dump(4);
    log().debug("Model saved to {}", filename);
}

nlohmann::json Model::serialize() const
{
    nlohmann::json res;
    nlohmann::json io;
    std::unordered_set<std::string> node_names;

    for (const auto& input : _input_nodes)
    {
        auto serialized = input->serialize();
        res["nodes"].push_back(serialized);
        io["input_nodes"].push_back(serialized.at("nodename"));
    }

    for (const auto& inter : _inter_nodes)
    {
        res["nodes"].push_back(inter->serialize());
    }

    for (const auto& output : _output_nodes)
    {
        auto serialized = output->serialize();
        io["output_nodes"].push_back(serialized.at("nodename"));
    }
    res["io"] = io;
    return res;
}

Model Model::load(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open file for reading.");
    }
    nlohmann::json j;
    file >> j;
    return deserialize(j);
}

Model Model::deserialize(nlohmann::json j)
{
    std::unordered_map<std::string, INode::PNode> all_nodes;
    std::string copy_word = "_copy";

    for (const auto& node_json : j["nodes"])
    {
        std::string nodename = node_json.at("nodename").get<std::string>() + copy_word;
        auto classname = node_json.at("classname").get<std::string>();
        INode::PNode node = INode::factory(classname, nodename);
        all_nodes.insert({nodename, node});
    }
    for (const auto& node_json : j["nodes"])
    {
        std::string nname = node_json.at("nodename").get<std::string>() + copy_word;
        all_nodes[nname]->deserialize(node_json, all_nodes);
    }

    std::vector<INode::PNode> input_nodes;
    std::vector<INode::PNode> output_nodes;
    for (const auto& input_name : j["io"]["input_nodes"])
    {
        auto input_node = all_nodes[input_name.get<std::string>() + copy_word];
        input_nodes.push_back(input_node);
    }
    for (const auto& output_name : j["io"]["output_nodes"])
    {
        auto output_node = all_nodes[output_name.get<std::string>() + copy_word];
        output_nodes.push_back(output_node);
    }
    return Model(input_nodes, output_nodes);
}