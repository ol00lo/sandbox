#include <catch2/catch.hpp>

#include "i_functions_node.hpp"
#include "arithmetic_nodes.hpp"
#include "arithmetic_op.hpp"
#include "power_nodes.hpp"
#include "input_node.hpp"
#include "power_op.hpp"
#include <iostream>

using namespace g;

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

    int a1_compute = 0, a2_compute = 0, a3_compute = 0, f_compute = 0;
    a1->add_value_callback([&a1_compute](IFunctionalNode*) {
        std::cout << "a1 node recomputed" << std::endl;
        a1_compute++;
    });
    a2->add_value_callback([&a2_compute](IFunctionalNode*) {
        std::cout << "a2 node recomputed" << std::endl;
        a2_compute++;
    });
    a3->add_value_callback([&a3_compute](IFunctionalNode*) {
        std::cout << "a3 node recomputed" << std::endl;
        a3_compute++;
    });
    f->add_value_callback([&f_compute](IFunctionalNode*) {
        std::cout << "f node recomputed" << std::endl;
        f_compute++;
    });

    A->set_value(1.2);
    x->set_value(2);
    B->set_value(-0.8);
    y->set_value(4);
    f->get_value();
    f->get_value();
    CHECK(a1_compute == 1);
    CHECK(a2_compute == 1);
    CHECK(a3_compute == 1);
    CHECK(f_compute == 1);
    y->set_value(5);
    f->get_value();
    CHECK(a1_compute == 1);
    CHECK(a2_compute == 1);
    CHECK(a3_compute == 2);
    CHECK(f_compute == 2);
    x->set_value(3);
    a2->get_value();
    CHECK(a1_compute == 2);
    CHECK(a2_compute == 2);
    CHECK(a3_compute == 2);
    CHECK(f_compute == 2);
    f->get_value();
    CHECK(a1_compute == 2);
    CHECK(a2_compute == 2);
    CHECK(a3_compute == 3);
    CHECK(f_compute == 3);
    A->set_value(1.3);
    f->get_value();
    CHECK(a1_compute == 3);
    CHECK(a2_compute == 3);
    CHECK(a3_compute == 4);
    CHECK(f_compute == 4);
}