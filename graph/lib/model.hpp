#ifndef MODEL_HPP
#define MODEL_HPP

#include "data_node.hpp"
#include "i_node.hpp"

namespace g
{
class Model
{
public:
    Model() {};
    Model(std::vector<INode::ptr_t> inputs, std::vector<INode::ptr_t> outputs);
    void save(const std::string& filename);
    static Model load(const std::string& filename);
    std::vector<Tensor> compute(const std::vector<Tensor>& input_values);
    const std::vector<INode::ptr_t> input_nodes() const;
    const std::vector<INode::ptr_t> inter_nodes() const;
    const std::vector<INode::ptr_t> output_nodes() const;
    std::vector<std::shared_ptr<DataNode>> param_nodes() const;
    void set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& p);

protected:
    std::vector<std::shared_ptr<DataNode>> param_nodes_;
    std::vector<INode::ptr_t> input_nodes_;
    std::vector<INode::ptr_t> output_nodes_;
    std::vector<INode::ptr_t> inter_nodes_;
    void add_into_inter(INode::ptr_t node);
};

} // namespace g

namespace nlohmann
{
template <>
struct adl_serializer<g::Model>
{
    static void to_json(json& j, const g::Model& model)
    {
        for (const auto& input : model.input_nodes())
        {
            j["nodes"].push_back(input);
            j["io"]["input_nodes"].push_back(input->nodename());
        }
        for (const auto& inter : model.inter_nodes())
        {
            j["nodes"].push_back(inter);
        }
        for (const auto& output : model.output_nodes())
        {
            j["io"]["output_nodes"].push_back(output->nodename());
        }
    }

    static void from_json(const json& j, g::Model& model)
    {
        std::unordered_map<std::string, g::INode::ptr_t> all_nodes;
        std::string copy_word = "_copy";

        for (const auto& node_json : j["nodes"])
        {
            g::INode::ptr_t node(node_json.get<g::INode::ptr_t>());
            all_nodes.insert({node->nodename(), node});
        }
        for (const auto& node_json : j["nodes"])
        {
            std::string nname = node_json.at("nodename").get<std::string>() + copy_word;
            all_nodes[nname]->set_dep(node_json, all_nodes);
        }

        std::vector<g::INode::ptr_t> input_nodes;
        std::vector<g::INode::ptr_t> output_nodes;
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
        model = g::Model(input_nodes, output_nodes);
    }
};
} // namespace nlohmann
#endif
