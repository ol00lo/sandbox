#ifndef ARITHMETIC_NODES_HPP
#define ARITHMETIC_NODES_HPP
#include "i_functional_node.hpp"

namespace g
{
class MultNode : public IFunctionalNode
{
public:
    MultNode(std::initializer_list<std::shared_ptr<INode>> args);

protected:
    double compute_value() override;
    std::string classname() const override;
    std::vector<double> get_gradient() override;

private:
    REGISTER_INODE_CHILD(MultNode);
};

class PlusNode : public IFunctionalNode
{
public:
    PlusNode(std::initializer_list<std::shared_ptr<INode>> args);

protected:
    double compute_value() override;
    std::string classname() const override;
    std::vector<double> get_gradient() override;

private:
    REGISTER_INODE_CHILD(PlusNode);
};

class MinusNode : public IFunctionalNode
{
public:
    MinusNode(std::initializer_list<std::shared_ptr<INode>> args);

protected:
    double compute_value() override;
    std::string classname() const override;
    std::vector<double> get_gradient() override;

private:
    REGISTER_INODE_CHILD(MinusNode);
};

} // namespace g
#endif