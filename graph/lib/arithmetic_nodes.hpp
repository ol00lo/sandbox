#ifndef ARITHMETIC_NODES_HPP
#define ARITHMETIC_NODES_HPP
#include "i_functional_node.hpp"

namespace g
{
class MultNode : public IFunctionalNode
{
public:
    MultNode(std::string nodename = "") : IFunctionalNode("MultNode", nodename){};

protected:
    Tensor compute_value() override;
    std::string classname() const override;
    std::vector<Tensor> gradient() override;

private:
    REGISTER_INODE_CHILD(MultNode);
};

class PlusNode : public IFunctionalNode
{
public:
    PlusNode(std::string nodename = "") : IFunctionalNode("PlusNode", nodename){};

protected:
    Tensor compute_value() override;
    std::string classname() const override;
    std::vector<Tensor> gradient() override;

private:
    REGISTER_INODE_CHILD(PlusNode);
};

class MinusNode : public IFunctionalNode
{
public:
    MinusNode(std::string nodename = "") : IFunctionalNode("MinusNode", nodename){};

protected:
    Tensor compute_value() override;
    std::string classname() const override;
    std::vector<Tensor> gradient() override;

private:
    REGISTER_INODE_CHILD(MinusNode);
};

} // namespace g
#endif