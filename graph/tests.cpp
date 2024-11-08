#include "catch.hpp"
#include "functional_nodes.hpp"
#include "graph.hpp"
#include <iostream>

TEST_CASE("test1", "[1]")
{
    std::shared_ptr<INode> x(new InputNode());
    std::shared_ptr<INode> y(new InputNode());
    std::shared_ptr<INode> A(new InputNode());
    std::shared_ptr<INode> B(new InputNode());

    std::shared_ptr<IFunctionalNode> a1 = op::mult(A, x);
    std::shared_ptr<IFunctionalNode> a2 = op::plus(a1, B);
    std::shared_ptr<IFunctionalNode> a3 = op::minus(y, a2);
    std::shared_ptr<IFunctionalNode> f = op::sqr(a3);

    A->set_value(1.2);
    x->set_value(2);
    B->set_value(-0.8);
    y->set_value(4);
    f->get_value();
    std::cout << f->get_value() << std::endl;
    y->set_value(5);
    std::cout << f->get_value() << std::endl;
    x->set_value(3);
    a2->get_value();
    std::cout << f->get_value() << std::endl;
    A->set_value(1.3);
    std::cout << f->get_value() << std::endl;
}