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

Model::Model(std::vector<pnode_t> inputs, std::vector<pnode_t> outputs) : input_nodes_(inputs), output_nodes_(outputs)
{
    for (const auto& node : input_nodes_)
    {
        save_name(node);
        nodes_.push_back(node);
    }
    for (const auto& output : output_nodes_)
    {
        add_into_inter(output);
    }
}

std::vector<Tensor> Model::compute(const std::vector<Tensor>& input_values)
{
    if (input_values.size() > input_nodes_.size())
    {
        throw std::runtime_error("Size of input values does not match the number of input nodes.");
    }
    for (size_t i = 0; i < input_values.size(); ++i)
    {
        INode* get_node = input_nodes_[i].get();
        if (DataNode* dataNode = dynamic_cast<DataNode*>(get_node))
        {
            dataNode->set_value(input_values[i]);
        }
    }

    std::vector<Tensor> results;
    for (const auto& output : output_nodes_)
    {
        results.push_back(output->value());
    }
    return results;
}
const std::vector<pnode_t> Model::nodes() const
{
    return nodes_;
}
const std::vector<pnode_t> Model::input_nodes() const
{
    return input_nodes_;
}
const std::vector<pnode_t> Model::output_nodes() const
{
    return output_nodes_;
}
std::vector<std::shared_ptr<DataNode>> Model::param_nodes() const
{
    return param_nodes_;
}
void Model::set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& p)
{
    for (int i = 0; i < param_nodes_.size(); i++)
    {
        param_nodes_[i]->set_value(p[i]->value());
    }
}
void Model::add_into_inter(pnode_t node)
{
    auto is_in_input = std::find(input_nodes_.begin(), input_nodes_.end(), node) != input_nodes_.end();
    if (is_in_input)
    {
        return;
    }
    bool not_in_inter = std::find(nodes_.begin(), nodes_.end(), node) == nodes_.end();
    if (not_in_inter)
    {
        save_name(node);
        nodes_.push_back(node);
        auto dataNode = std::dynamic_pointer_cast<DataNode>(node);
        if (dataNode != nullptr)
        {
            param_nodes_.push_back(dataNode);
        }
    }
    for (const auto& prev : node->prev_nodes())
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
    log().info("Model saved to {}", filename);
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
    Model res = j.get<Model>();
    log().info("Model loaded from {}", filename);
    return res;
}

void Model::save_name(pnode_t node)
{
    std::string nodename = node->nodename();
    if (names_.find(nodename) == names_.end())
        names_.insert(nodename);
    else
        node->rename();
}