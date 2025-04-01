#ifndef INPUT_NODE_HPP
#define INPUT_NODE_HPP
#include "i_node.hpp"

namespace g
{
class DataNode : public INode
{
public:
    DataNode(std::string nodename) : INode("DataNode",  nodename), value_({0}) {};
    void serialize_spec(nlohmann::json& js) const override;
    void deserialize_spec(const nlohmann::json&) override;
    Tensor value() override;
    void set_value(Tensor val);
    Tensor notself_derivative(const INode* arg) override;
    std::string classname() const override;
    Shape output_shape() const override;

private:
    Tensor value_;
    REGISTER_INODE_CHILD(DataNode);
};
} // namespace g
#endif
