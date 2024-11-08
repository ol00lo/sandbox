#include "functional_nodes.hpp"

std::shared_ptr<IFunctionalNode> op::mult(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    std::shared_ptr<IFunctionalNode> a(new MultNode);
    a->add_prev(a1);
    a->add_prev(a2);
    a1->add_next(a);
    a2->add_next(a);
    return a;
}

std::shared_ptr<IFunctionalNode> op::plus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    std::shared_ptr<IFunctionalNode> a(new PlusNode);
    a->add_prev(a1);
    a->add_prev(a2);
    a1->add_next(a);
    a2->add_next(a);
    return a;
}

std::shared_ptr<IFunctionalNode> op::minus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2)
{
    std::shared_ptr<IFunctionalNode> a(new MinusNode);
    a->add_prev(a1);
    a->add_prev(a2);
    a1->add_next(a);
    a2->add_next(a);
    return a;
}
std::shared_ptr<IFunctionalNode> op::sqr(std::shared_ptr<INode> a)
{
    std::shared_ptr<IFunctionalNode> b(new SqrNode);
    b->add_prev(a);
    a->add_next(b);
    return b;
}