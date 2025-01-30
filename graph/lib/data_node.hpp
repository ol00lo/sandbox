#ifndef INPUT_NODE_HPP
#define INPUT_NODE_HPP
#include "i_node.hpp"

namespace g
{
class DataNode : public INode
{
public:
    DataNode(std::string nodename) : INode(nodename), _value({0}) {};
    static std::shared_ptr<DataNode> factory(std::string classname, std::string nodename = "")
    {
        if (classname != "DataNode")
        {
            throw std::runtime_error("Invalid classname: " + classname);
        }
        return std::static_pointer_cast<DataNode>(std::make_shared<DataNode>(nodename));
    }

    void serialize_spec(nlohmann::json& js) const override;
    Tensor get_value() override;
    void set_value(Tensor val);
    Tensor notself_derivative(const INode* arg) override;
    std::string classname() const override;
    Shape get_shape() const override;

private:
    Tensor _value;
    REGISTER_INODE_CHILD(DataNode);
};
} // namespace g
#endif
