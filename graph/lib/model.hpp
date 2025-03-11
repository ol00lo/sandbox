#ifndef MODEL_HPP
#define MODEL_HPP

#include "data_node.hpp"
#include "i_node.hpp"

namespace g
{
class Model
{
public:
    Model(std::vector<INode::ptr_t> inputs, std::vector<INode::ptr_t> outputs);
    void save(const std::string& filename);
    nlohmann::json serialize() const;
    static Model load(const std::string& filename);
    static Model deserialize(nlohmann::json);
    std::vector<Tensor> compute(const std::vector<Tensor>& input_values);
    std::vector<INode::ptr_t> get_input_nodes() const;
    std::vector<INode::ptr_t> get_inter_nodes() const;
    std::vector<INode::ptr_t> get_output_nodes() const;
    std::vector<std::shared_ptr<DataNode>> get_param_nodes() const;
    void set_param(const std::vector<std::shared_ptr<DataNode>>& p);

protected:
    std::vector<std::shared_ptr<DataNode>> _param_nodes;
    std::vector<INode::ptr_t> _input_nodes;
    std::vector<INode::ptr_t> _output_nodes;
    std::vector<INode::ptr_t> _inter_nodes;
    void add_into_inter(INode::ptr_t node);
    bool is_in_param(const DataNode* node);
};

} // namespace g
#endif