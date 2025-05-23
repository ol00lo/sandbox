#ifndef POWER_NODES_HPP
#define POWER_NODES_HPP
#include "i_functional_node.hpp"

namespace g
{
class SqrNode : public IFunctionalNode
{
public:
    SqrNode(std::string nodename = "") : IFunctionalNode("SqrNode", nodename){};

protected:
    Tensor compute_value() override;
    std::string classname() const override;
    std::vector<Tensor> gradient() override;

private:
    REGISTER_INODE_CHILD(SqrNode);
};
} // namespace g
#endif