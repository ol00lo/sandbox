#include "arithmetic_op.hpp"
#include "arithmetic_nodes.hpp"

using namespace g;

std::shared_ptr<IFunctionalNode> op::mult(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    std::shared_ptr<IFunctionalNode> a(new MultNode(a1, a2));
    a1->add_next(a);
    a2->add_next(a);
    return a;
}

std::shared_ptr<IFunctionalNode> op::plus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    std::shared_ptr<IFunctionalNode> a(new PlusNode(a1, a2));
    a1->add_next(a);
    a2->add_next(a);
    return a;
}

std::shared_ptr<IFunctionalNode> op::minus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    std::shared_ptr<IFunctionalNode> a(new MinusNode(a1, a2));
    a1->add_next(a);
    a2->add_next(a);
    return a;
}