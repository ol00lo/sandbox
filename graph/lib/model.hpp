#ifndef MODEL_HPP
#define MODEL_HPP

#include "data_node.hpp"
#include "i_node.hpp"

namespace g
{
class Model
{
public:
    Model(std::vector<INode::PNode> inputs, std::vector<INode::PNode> outputs);
    Model() {}
    void save(const std::string& filename);
    nlohmann::json serialize() const;
    static Model load(const std::string& filename);
    static Model deserialize(nlohmann::json);
    std::vector<Tensor> compute(const std::vector<Tensor>& input_values);
    std::vector<INode::PNode> get_input_nodes() const;
    std::vector<INode::PNode> get_inter_nodes() const;
    std::vector<INode::PNode> get_output_nodes() const;
    std::vector<std::shared_ptr<DataNode>> get_param_nodes() const;
    void set_param(const std::vector<std::shared_ptr<DataNode>>& p);

protected:
    std::vector<std::shared_ptr<DataNode>> _param_nodes;
    std::vector<INode::PNode> _input_nodes;
    std::vector<INode::PNode> _output_nodes;
    std::vector<INode::PNode> _inter_nodes;
    void add_into_inter(INode::PNode node);
    bool is_in_param(DataNode* node);
};

} // namespace g
#endif