#include <catch2/catch.hpp>

#include "arithmetic_nodes.hpp"
#include "i_functions_node.hpp"
#include "input_node.hpp"
#include "power_nodes.hpp"
#include <iostream>

using namespace g;

TEST_CASE("test1", "[1]")
{
    std::shared_ptr<InputNode> x(new InputNode());
    std::shared_ptr<InputNode> y(new InputNode());
    std::shared_ptr<InputNode> A(new InputNode());
    std::shared_ptr<InputNode> B(new InputNode());

    std::shared_ptr<IFunctionalNode> a1 = op::mult(A, x);
    std::shared_ptr<IFunctionalNode> a2 = op::plus(a1, B);
    std::shared_ptr<IFunctionalNode> a3 = op::minus(y, a2);
    std::shared_ptr<IFunctionalNode> f = op::sqr(a3);

    int a1_compute = 0, a2_compute = 0, a3_compute = 0, f_compute = 0;
    a1->add_value_callback([&a1_compute](IFunctionalNode*) {
        a1_compute++;
    });
    a2->add_value_callback([&a2_compute](IFunctionalNode*) {
        a2_compute++;
    });
    a3->add_value_callback([&a3_compute](IFunctionalNode*) {
        a3_compute++;
    });
    f->add_value_callback([&f_compute](IFunctionalNode*) {
        f_compute++;
    });

    log()->info("change A: ");
    A->set_value(1.2);
    log()->info("change x: ");
    x->set_value(2);
    log()->info("change B: ");
    B->set_value(-0.8);
    log()->info("change y: ");
    y->set_value(4);
    CHECK(f->get_value() == Approx(5.76));
    CHECK(f->get_value() == Approx(5.76));
    CHECK(a1_compute == 1);
    CHECK(a2_compute == 1);
    CHECK(a3_compute == 1);
    CHECK(f_compute == 1);

    log()->info("change y: ");
    y->set_value(5);
    CHECK(f->get_value() == Approx(11.56));
    CHECK(a1_compute == 1);
    CHECK(a2_compute == 1);
    CHECK(a3_compute == 2);
    CHECK(f_compute == 2);

    log()->info("change x: ");
    x->set_value(3);
    CHECK(a2->get_value() == Approx(2.8));
    CHECK(a1_compute == 2);
    CHECK(a2_compute == 2);
    CHECK(a3_compute == 2);
    CHECK(f_compute == 2);
    CHECK(f->get_value() == Approx(4.84));
    CHECK(a1_compute == 2);
    CHECK(a2_compute == 2);
    CHECK(a3_compute == 3);
    CHECK(f_compute == 3);

    log()->info("change A: ");
    A->set_value(1.3);
    CHECK(f->get_value() == Approx(3.61));
    CHECK(a1_compute == 3);
    CHECK(a2_compute == 3);
    CHECK(a3_compute == 4);
    CHECK(f_compute == 4);
}