#ifndef ARITHMETIC_NODES_HPP
#define ARITHMETIC_NODES_HPP
#include "i_functions_node.hpp"

namespace g
{
class MultNode : public IFunctionalNode
{
protected:
    double compute_value() override;
};

class PlusNode : public IFunctionalNode
{
protected:
    double compute_value() override;
};

class MinusNode : public IFunctionalNode
{
protected:
    double compute_value() override;
};

namespace op
{
std::shared_ptr<IFunctionalNode> mult(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
std::shared_ptr<IFunctionalNode> plus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
std::shared_ptr<IFunctionalNode> minus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
} // namespace op

} // namespace g
#endif