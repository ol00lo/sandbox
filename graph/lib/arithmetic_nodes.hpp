#ifndef ARITHMETIC_NODES_HPP
#define ARITHMETIC_NODES_HPP
#include "i_functional_node.hpp"

namespace g
{
class MultNode : public IFunctionalNode
{
public:
    MultNode(std::string nodename = "") : IFunctionalNode(nodename) {};
    void set_prev_nodes(std::initializer_list<std::shared_ptr<INode>> args) override;
protected:
    double compute_value() override;
    std::string classname() const override;
    static std::string classname_static();
    std::vector<double> get_gradient() override;

private:
    REGISTER_INODE_CHILD(MultNode);
};

class PlusNode : public IFunctionalNode
{
public:
    PlusNode(std::string nodename = "") : IFunctionalNode(nodename) {};
    void set_prev_nodes(std::initializer_list<std::shared_ptr<INode>> args) override;

protected:
    double compute_value() override;
    std::string classname() const override;
    static std::string classname_static();
    std::vector<double> get_gradient() override;

private:
    REGISTER_INODE_CHILD(PlusNode);
};

class MinusNode : public IFunctionalNode
{
public:
    MinusNode(std::string nodename = "") : IFunctionalNode(nodename) {};
    void set_prev_nodes(std::initializer_list<std::shared_ptr<INode>> args) override;

protected:
    double compute_value() override;
    std::string classname() const override;
    static std::string classname_static();
    std::vector<double> get_gradient() override;

private:
    REGISTER_INODE_CHILD(MinusNode);
};

} // namespace g
#endif