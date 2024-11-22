#ifndef ARITHMETIC_NODES_HPP
#define ARITHMETIC_NODES_HPP
#include "i_functional_node.hpp"

namespace g
{
class MultNode : public IFunctionalNode
{
protected:
    double compute_value() override;
    std::string classname() const override;
    std::vector<double> get_gradient() override;
};

class PlusNode : public IFunctionalNode
{
protected:
    double compute_value() override;
    std::string classname() const override;
    std::vector<double> get_gradient() override;
};

class MinusNode : public IFunctionalNode
{
protected:
    double compute_value() override;
    std::string classname() const override;
    std::vector<double> get_gradient() override;
};

namespace op
{
std::shared_ptr<IFunctionalNode> mult(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
std::shared_ptr<IFunctionalNode> plus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
std::shared_ptr<IFunctionalNode> minus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
} // namespace op

} // namespace g
#endif