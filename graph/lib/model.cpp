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
}

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
    if (node->classname()  == InputNode::classname_static())
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
    for (const auto& input : _input_nodes)
    {
        res["input_nodes"].push_back(input->serialize());
    }

    for (const auto& inter : _inter_nodes)
    {
        res["inter_nodes"].push_back(inter->serialize());
    }

    for (const auto& output : _output_nodes)
    {
        res["output_nodes"].push_back(output->serialize());
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
    std::unordered_map<std::shared_ptr<INode>, std::vector<std::string>> inter_nodes;

    for (const auto& node_json : j["input_nodes"])
    {
        auto input_node = INode::factory(node_json.at("classname").get<std::string>(), {});
        all_nodes.insert({node_json.at("nodename").get<std::string>(), input_node});
        input_nodes.push_back(input_node);
    }
    for (const auto& node_json : j["inter_nodes"])
    {
        auto inter_node = INode::factory(node_json.at("classname").get<std::string>(), {});
        all_nodes.insert({node_json.at("nodename").get<std::string>(), inter_node});
        if (node_json.contains("prev_nodes"))
        {
            for (const auto& prev_node_name : node_json.at("prev_nodes"))
            {
                inter_nodes[inter_node].push_back(prev_node_name.get<std::string>());
            }
        }
    }
    for (const auto& node : inter_nodes)
    {
        for (int i = 0; i < node.second.size(); i++)
        {
            auto prev_node = find_node_by_name(all_nodes, node.second[i]);
            node.first->add_prev(prev_node);
            prev_node->add_next(node.first);
        }
    }

    for (const auto& node_json : j["output_nodes"])
    {
        auto output_node = INode::factory(node_json.at("classname").get<std::string>(), {});
        if (node_json.contains("prev_nodes"))
        {
            for (const auto& prev_node_name : node_json.at("prev_nodes"))
            {
                auto prev_node = find_node_by_name(all_nodes, prev_node_name.get<std::string>());
                if (prev_node)
                {
                    output_node->add_prev(prev_node);
                }
            }
        }
        output_nodes.push_back(output_node);
    }

    return Model(input_nodes, output_nodes);
}