#ifndef TRIGONOMETRIC_NODES_HPP
#define TRIGONOMETRIC_NODES_HPP
#include "i_functional_node.hpp"

namespace g
{
class SinNode : public IFunctionalNode
{
public:
    SinNode(std::string nodename = "") : IFunctionalNode(nodename) {};

protected:
    Tensor compute_value() override;
    std::string classname() const override;
    std::vector<Tensor> gradient() override;

private:
    REGISTER_INODE_CHILD(SinNode);
};

class CosNode : public IFunctionalNode
{
public:
    CosNode(std::string nodename = "") : IFunctionalNode(nodename) {};

protected:
    Tensor compute_value() override;
    std::string classname() const override;
    std::vector<Tensor> gradient() override;

private:
    REGISTER_INODE_CHILD(CosNode);
};

} // namespace g
#endif