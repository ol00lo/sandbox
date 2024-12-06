#ifndef POWER_NODES_HPP
#define POWER_NODES_HPP
#include "i_functional_node.hpp"

namespace g
{
class SqrNode : public IFunctionalNode
{
public:
    SqrNode(std::initializer_list<std::shared_ptr<INode>> args);

protected:
    double compute_value() override;
    std::string classname() const override;
    std::vector<double> get_gradient() override;

private:
    REGISTER_INODE_CHILD(SqrNode);
};
} // namespace g
#endif