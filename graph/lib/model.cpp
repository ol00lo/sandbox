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
using pnode_t = INode::ptr_t;
pnode_t find_node_by_name(std::unordered_map<std::string, pnode_t> all_nodes, const std::string& name)
{
    auto it = all_nodes.find(name);
    if (it != all_nodes.end())
    {
        return it->second;
    }
    return nullptr;
}
} // namespace

Model::Model(std::vector<pnode_t> inputs, std::vector<pnode_t> outputs) : _input_nodes(inputs), _output_nodes(outputs)
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
const std::vector<pnode_t> Model::get_input_nodes() const
{
    return _input_nodes;
}
const std::vector<pnode_t> Model::get_inter_nodes() const
{
    return _inter_nodes;
}
const std::vector<pnode_t> Model::get_output_nodes() const
{
    return _output_nodes;
}
std::vector<std::shared_ptr<DataNode>> Model::get_param_nodes() const
{
    return _param_nodes;
}
void Model::set_param(const std::vector<std::shared_ptr<DataNode>>& p)
{
    for (int i = 0; i < _param_nodes.size(); i++)
    {
        _param_nodes[i]->set_value(p[i]->get_value());
    }
}
void Model::add_into_inter(pnode_t node)
{
    auto is_in_input = std::find(_input_nodes.begin(), _input_nodes.end(), node) != _input_nodes.end();
    if (is_in_input)
    {
        return;
    }
    bool not_in_inter = std::find(_inter_nodes.begin(), _inter_nodes.end(), node) == _inter_nodes.end();
    if (not_in_inter)
    {
        _inter_nodes.push_back(node);
        auto dataNode = std::dynamic_pointer_cast<DataNode>(node);
        if (dataNode != nullptr)
        {
            _param_nodes.push_back(dataNode);
        }
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
    nlohmann::json js = *this;
    file << js.dump(4);
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
    return j.get<Model>();
}
