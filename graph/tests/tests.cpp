#include "arithmetic_nodes.hpp"
#include "i_functional_node.hpp"
#include "input_node.hpp"
#include "power_nodes.hpp"
#include <catch2/catch.hpp>
#include <iostream>

using namespace g;

TEST_CASE("test1", "[1]")
{
    g::set_log_debug();
    std::shared_ptr<InputNode> x(new InputNode());
    std::shared_ptr<InputNode> y(new InputNode());
    std::shared_ptr<InputNode> A(new InputNode());
    std::shared_ptr<InputNode> B(new InputNode());

    std::shared_ptr<IFunctionalNode> a1 = op::mult(A, x);
    std::shared_ptr<IFunctionalNode> a2 = op::plus(a1, B);
    std::shared_ptr<IFunctionalNode> a3 = op::minus(y, a2);
    std::shared_ptr<IFunctionalNode> f = op::sqr(a3);

    int a1_compute = 0, a2_compute = 0, a3_compute = 0, f_compute = 0;
    a1->add_value_callback([&a1_compute](IFunctionalNode*) { a1_compute++; });
    a2->add_value_callback([&a2_compute](IFunctionalNode*) { a2_compute++; });
    a3->add_value_callback([&a3_compute](IFunctionalNode*) { a3_compute++; });
    f->add_value_callback([&f_compute](IFunctionalNode*) { f_compute++; });

    A->set_value(1.2);
    x->set_value(2);
    B->set_value(-0.8);
    y->set_value(4);
    CHECK(f->get_value() == Approx(5.76));
    CHECK(f->get_value() == Approx(5.76));
    CHECK(a1_compute == 1);
    CHECK(a2_compute == 1);
    CHECK(a3_compute == 1);
    CHECK(f_compute == 1);

    y->set_value(5);
    CHECK(f->get_value() == Approx(11.56));
    CHECK(a1_compute == 1);
    CHECK(a2_compute == 1);
    CHECK(a3_compute == 2);
    CHECK(f_compute == 2);

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

    A->set_value(1.3);
    CHECK(f->get_value() == Approx(3.61));
    CHECK(a1_compute == 3);
    CHECK(a2_compute == 3);
    CHECK(a3_compute == 4);
    CHECK(f_compute == 4);
}

TEST_CASE("derivative", "[2]")
{
    g::set_log_debug();
    std::shared_ptr<InputNode> x(new InputNode());
    std::shared_ptr<InputNode> y(new InputNode());
    std::shared_ptr<InputNode> A(new InputNode());
    std::shared_ptr<InputNode> B(new InputNode());

    std::shared_ptr<IFunctionalNode> a1 = op::mult(A, x);
    std::shared_ptr<IFunctionalNode> a2 = op::plus(a1, B);
    std::shared_ptr<IFunctionalNode> a3 = op::minus(y, a2);
    std::shared_ptr<IFunctionalNode> f = op::sqr(a3);
    int a1_gradient = 0, a2_gradient = 0, a3_gradient = 0, f_gradient = 0;
    a1->add_gradient_callback([&a1_gradient](IFunctionalNode*) { a1_gradient++; });
    a2->add_gradient_callback([&a2_gradient](IFunctionalNode*) { a2_gradient++; });
    a3->add_gradient_callback([&a3_gradient](IFunctionalNode*) { a3_gradient++; });
    f->add_gradient_callback([&f_gradient](IFunctionalNode*) { f_gradient++; });

    A->set_value(1.2);
    B->set_value(-0.8);
    x->set_value(2);
    y->set_value(3);
    CHECK(f->get_derivative(A) == Approx(-5.6));
    CHECK(f->get_derivative(B) == Approx(-2.8));
    CHECK(a1_gradient == 1);
    CHECK(a2_gradient == 1);
    CHECK(a3_gradient == 1);
    CHECK(f_gradient == 1);
    y->set_value(1);
    CHECK(f->get_derivative(A) == Approx(2.4));
    CHECK(f->get_derivative(B) == Approx(1.2));
    CHECK(a1_gradient == 1);
    CHECK(a2_gradient == 1);
    CHECK(a3_gradient == 2);
    CHECK(f_gradient == 2);
    x->set_value(4);
    CHECK(f->get_derivative(A) == Approx(24));
    CHECK(f->get_derivative(B) == Approx(6));
    CHECK(a1_gradient == 2);
    CHECK(a2_gradient == 2);
    CHECK(a3_gradient == 3);
    CHECK(f_gradient == 3);
}