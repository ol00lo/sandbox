#include "data_node.hpp"
#include "graph.hpp"
#include "i_functional_node.hpp"
#include "model.hpp"
#include "tensor.hpp"
#include <catch2/catch.hpp>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace g;

TEST_CASE("index tetst", "[index_test]")
{
    Shape sh1(1, 1, 2, 3);
    Shape sh2(2, 3);
    Index i1(2, sh1);
    Index i2(4, sh1);
    CHECK(sh2 == sh1);
    CHECK(i1[0] == 0);
    CHECK(i1[1] == 0);
    CHECK(i1[2] == 0);
    CHECK(i1[3] == 2);
    CHECK(i2[0] == 0);
    CHECK(i2[1] == 0);
    CHECK(i2[2] == 1);
    CHECK(i2[3] == 1);
    CHECK(i1.to_linear(sh1) == 2);
    CHECK(i2.to_linear(sh2) == 4);
}

TEST_CASE("tensor test", "[tensor_test]")
{
    Tensor t1({1, 2, 3});
    Tensor t2(Shape(3), {1, 2, 3});
    Tensor t3(Shape(1, 1, 3), {1, 2, 3});
    Tensor t4(Shape(1, 1, 3), 0.0);

    CHECK(t1 == t2);
    CHECK(t3 == t2);
    t3.set_zero();
    CHECK(t3 == t4);
    Tensor t5({3, 4, 5});
    t1.add(t5);
    CHECK(t1[0] == 4);
    CHECK(t1[1] == 6);
    CHECK(t1[2] == 8);
    Tensor t6 = mult(t1, t2);
    CHECK(t6[0] == 4);
    CHECK(t6[1] == 12);
    CHECK(t6[2] == 24);
    t6.div(t5);
    CHECK(t6[1] == 3);
}

TEST_CASE("compute test", "[compute_test]")
{
    Tensor a1(Shape(1), {0});
    Tensor a2(Shape(2), {1, 1});
    CHECK_THROWS(g::mult(a1, a2));
    CHECK_THROWS(g::add(a1, a2));
    a1.apply_oper([](double x) { return std::sin(x); });
    CHECK(a1[0] == Approx(0.0));
    CHECK(g::sin(a1)[0] == Approx(0.0));
    CHECK(g::cos(a1)[0] == Approx(1.0));
    CHECK(g::tg(a1)[0] == Approx(0.0));
    CHECK_THROWS(g::ctg(a1));
    CHECK(g::cos(a2)[0] == g::cos(a2)[1]);
}

namespace
{
size_t n_braces(std::string s)
{
    size_t res = 0;
    auto i = s.find("[");
    while (i != std::string::npos)
    {
        s.erase(0, i + 1);
        i = s.find("[");
        res++;
    }
    return res;
}
} // namespace
TEST_CASE("print tensor test", "[print_tensor_test]")
{
    auto shape = Shape(1, 1, 3);
    Tensor x(shape, {1, 2, 3});
    CHECK(shape.n_indexes() == 3);
    std::ostringstream oss1;
    x.write(oss1);
    CHECK(n_braces(oss1.str()) == 3);
    Tensor x1(Shape(2, 2, 1), {1, 2, 3, 4});
    std::ostringstream oss;
    x1.write(oss);
    CHECK(n_braces(oss.str()) == 7);
    CHECK(oss.str()[11] == '1');
}