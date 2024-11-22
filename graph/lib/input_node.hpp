#ifndef INPUT_NODE_HPP
#define INPUT_NODE_HPP
#include "i_node.hpp"

namespace g
{
class InputNode : public INode
{
public:
    double get_value() override;
    void set_value(double val);
    double notself_derivative(const INode* arg) override;

private:
    double _value = 0;

    std::string classname() const override;
};
} // namespace g
#endif
