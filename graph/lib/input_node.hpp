#ifndef INPUT_NODE_HPP
#define INPUT_NODE_HPP
#include "i_node.hpp"

namespace g
{
class InputNode : public INode
{
public:
    InputNode() = default;
    InputNode(std::initializer_list<std::shared_ptr<INode>> args)
    {
    }
    nlohmann::json serialize() const override;
    double get_value() override;
    void set_value(double val);
    double notself_derivative(const INode* arg) override;
    std::string classname() const override;

private:
    double _value = 0;
    REGISTER_INODE_CHILD(InputNode);
};
} // namespace g
#endif
