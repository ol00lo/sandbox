#ifndef MODEL_HPP
#define MODEL_HPP

#include "data_node.hpp"
#include "i_node.hpp"
#include <unordered_set>

namespace g
{
using pnode_t = INode::ptr_t;

class Model
{
public:
    Model() {};
    Model(std::vector<pnode_t> inputs, std::vector<pnode_t> outputs);

    static Model load(const std::string& filename);
    void save(const std::string& filename);

    const std::vector<pnode_t> nodes() const;
    const std::vector<pnode_t> input_nodes() const;
    const std::vector<pnode_t> output_nodes() const;
    std::vector<std::shared_ptr<DataNode>> param_nodes() const;

    void set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& p);

    std::vector<Tensor> compute(const std::vector<Tensor>& input_values);

protected:
    std::unordered_set<std::string> names_;
    std::vector<pnode_t> nodes_;
    std::vector<pnode_t> input_nodes_;
    std::vector<pnode_t> output_nodes_;
    std::vector<std::shared_ptr<DataNode>> param_nodes_;
    void add_into_inter(pnode_t node);
    void save_name(pnode_t node);
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
        for (const auto& inter : model.nodes())
        {
            j["nodes"].push_back(inter);
        }
        for (const auto& output : model.output_nodes())
        {
            j["io"]["output_nodes"].push_back(output->nodename());
        }
        g::log().debug("Model serialized");
    }

    static void from_json(const json& j, g::Model& model)
    {
        std::unordered_map<std::string, g::pnode_t> all_nodes;

        for (const auto& node_json : j["nodes"])
        {
            g::INode::ptr_t node(node_json.get<g::pnode_t>());
            all_nodes.insert({node->nodename(), node});
        }
        for (const auto& node_json : j["nodes"])
        {
            std::string nname = node_json.at("nodename").get<std::string>();
            all_nodes[nname]->set_dep(node_json, all_nodes);
        }

        std::vector<g::pnode_t> input_nodes;
        std::vector<g::pnode_t> output_nodes;
        for (const auto& input_name : j["io"]["input_nodes"])
        {
            auto input_node = all_nodes[input_name.get<std::string>()];
            input_nodes.push_back(input_node);
        }
        for (const auto& output_name : j["io"]["output_nodes"])
        {
            auto output_node = all_nodes[output_name.get<std::string>()];
            output_nodes.push_back(output_node);
        }
        model = g::Model(input_nodes, output_nodes);
		g::log().debug("Model deserialized");
    }
};
} // namespace nlohmann
#endif
