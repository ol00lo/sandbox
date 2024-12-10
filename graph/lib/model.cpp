#include "model.hpp"
#include "arithmetic_nodes.hpp"
#include "power_nodes.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace g;
namespace
{
std::shared_ptr<INode> find_node_by_name(std::unordered_map<std::string, std::shared_ptr<INode>> all_nodes,
                                         const std::string& name)
{
    auto it = all_nodes.find(name);
    if (it != all_nodes.end())
    {
        return it->second;
    }
    return nullptr;
}
} // namespace

Model::Model(std::vector<std::shared_ptr<INode>> inputs, std::vector<std::shared_ptr<INode>> outputs)
    : _input_nodes(std::move(inputs)), _output_nodes(std::move(outputs))
{
    for (const auto& output : _output_nodes)
    {
        add_into_inter(output);
    }
}

std::vector<double> Model::compute(const std::vector<double>& input_values)
{
    if (input_values.size() != _input_nodes.size())
    {
        throw std::runtime_error("Size of input values does not match the number of input nodes.");
    }
    for (size_t i = 0; i < _input_nodes.size(); ++i)
    {
        _input_nodes[i]->set_value(input_values[i]);
    }

    std::vector<double> results;
    for (const auto& output : _output_nodes)
    {
        results.push_back(output->get_value());
    }
    return results;
}

void Model::add_into_inter(std::shared_ptr<INode> node)
{
    if (node->classname() == "InputNode")
    {
        return;
    }
    if (std::find(_inter_nodes.begin(), _inter_nodes.end(), node) == _inter_nodes.end())
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
    std::unordered_set<std::string> node_names;

    for (const auto& input : _input_nodes)
    {
        auto serialized = input->serialize();
        res["nodes"].push_back(serialized);
        res["input_nodes"].push_back(serialized.at("nodename"));
    }

    for (const auto& inter : _inter_nodes)
    {
        res["nodes"].push_back(inter->serialize());
    }

    for (const auto& output : _output_nodes)
    {
        auto serialized = output->serialize();
        res["nodes"].push_back(serialized);
        res["output_nodes"].push_back(serialized.at("nodename"));
    }

    return res;
}

Model Model::load(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open file for reading.");
    }
    std::unordered_map<std::string, std::shared_ptr<INode>> all_nodes;
    nlohmann::json j;
    file >> j;
    return deserialize(j);
}

Model Model::deserialize(nlohmann::json j)
{
    std::unordered_map<std::string, std::shared_ptr<INode>> all_nodes;
    std::vector<std::shared_ptr<INode>> input_nodes;
    std::vector<std::shared_ptr<INode>> output_nodes;

    for (const auto& node_json : j["nodes"])
    {
        auto node = INode::factory(node_json.at("classname").get<std::string>(), {});
        auto name = node_json.at("nodename").get<std::string>();
        all_nodes.insert({name, node});
    }
    for (const auto& node_json : j["nodes"])
    {
        auto node = all_nodes[node_json.at("nodename").get<std::string>()];
        if (!node_json.at("prev_nodes").empty())
        {
            for (const auto& prev_node_name : node_json.at("prev_nodes"))
            {
                auto prev_node = all_nodes[prev_node_name.get<std::string>()];
                node->add_prev(prev_node);
                prev_node->add_next(node);
            }
        }
    }



    for (const auto& input_name : j["input_nodes"])
    {
        auto input_node = all_nodes[input_name.get<std::string>()];
        input_nodes.push_back(input_node);
    }

    for (const auto& output_name : j["output_nodes"])
    {
        auto output_node = all_nodes[output_name.get<std::string>()];
        output_nodes.push_back(output_node);
    }

    return Model(input_nodes, output_nodes);
}