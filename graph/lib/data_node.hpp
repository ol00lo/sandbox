#ifndef INPUT_NODE_HPP
#define INPUT_NODE_HPP
#include "i_node.hpp"

namespace g
{
class DataNode : public INode
{
public:
    DataNode(std::string nodename) : INode(nodename), _value({0}) {};
    void serialize_spec(nlohmann::json& js) const override;
    Tensor get_value() override;
    void set_value(Tensor val);
    Tensor notself_derivative(const INode* arg) override;
    std::string classname() const override;

private:
    Tensor _value;
    REGISTER_INODE_CHILD(DataNode);
};
} // namespace g
#endif
