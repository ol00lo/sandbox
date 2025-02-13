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

void Model::add_into_inter(std::shared_ptr<INode> node)
{
    auto fnd = std::find(_input_nodes.begin(), _input_nodes.end(), node);
    if (fnd != _input_nodes.end())
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
    nlohmann::json js = *this;
    file << js.dump(4);
    log().debug("Model saved to {}", filename);
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
    return Model(j.get<Model>());
}
const std::vector<INode::ptr_t>& Model::input_nodes() const
{
    return _input_nodes;
}
const std::vector<INode::ptr_t>& Model::output_nodes() const
{
    return _output_nodes;
}
const std::vector<INode::ptr_t>& Model::inter_nodes() const
{
    return _inter_nodes;
}