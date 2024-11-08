#ifndef ARITHMETIC_NODES_HPP
#define ARITHMETIC_NODES_HPP
#include "i_functions_node.hpp"
namespace g
{
class MultNode : public IFunctionalNode
{
public:
    MultNode(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);

protected:
    double compute_value() override;
};

class PlusNode : public IFunctionalNode
{
public:
    PlusNode(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);

protected:
    double compute_value() override;
};

class MinusNode : public IFunctionalNode
{
public:
    MinusNode(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);

protected:
    double compute_value() override;
};
} // namespace g
#endif