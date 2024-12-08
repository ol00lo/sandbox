#ifndef INPUT_NODE_HPP
#define INPUT_NODE_HPP
#include "i_node.hpp"

namespace g
{
class InputNode : public INode
{
public:
    InputNode(std::string nodename) : INode(nodename) {};
    void serialize_spec(nlohmann::json& js) const override;
    double get_value() override;
    void set_value(double val);
    double notself_derivative(const INode* arg) override;
    std::string classname() const override;
    static std::string classname_static();

private:
    double _value = 0;
    REGISTER_INODE_CHILD(InputNode);
};
} // namespace g
#endif
