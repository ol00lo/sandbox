#include "data_node.hpp"
#include "graph.hpp"
#include "i_functional_node.hpp"
#include "model.hpp"
#include "tensor.hpp"
#include <catch2/catch.hpp>
#include <fstream>
#include <iostream>

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
TEST_CASE("graph", "[graph]")
{
    auto shape = Shape(1, 1, 3);
    Tensor x(shape, {1, 2, 3});
    Tensor y(shape, {4, 5, 6});

    std::shared_ptr<DataNode> x_node = DataNode::factory("DataNode", "x");
    std::shared_ptr<DataNode> y_node = DataNode::factory("DataNode", "y");
    std::shared_ptr<DataNode> dva_node = DataNode::factory("DataNode", "dva");

    x_node->set_value(x);
    y_node->set_value(y);
    dva_node->set_value(Tensor(shape, 2));

    std::shared_ptr<INode> a1 = INode::factory("MultNode", "dva_x");
    g::set_dep(a1, {dva_node, x_node});
    std::shared_ptr<INode> a2 = INode::factory("SqrNode", "sqr_y");
    g::set_dep(a2, {y_node});
    std::shared_ptr<INode> f_node = INode::factory("PlusNode", "f_node");
    g::set_dep(f_node, {a1, a2});

	Model model({x_node, y_node}, {f_node});
    model.save("ssss.json");
    Model model2 = Model::load("ssss.json");
    std::vector<Tensor> res = model.compute({});
    std::vector<Tensor> res1 = model2.compute({});
    CHECK(res[0] == res1[0]);

    auto gx = model.compute_derivative(f_node, x_node);
    auto gy = model.compute_derivative(f_node, y_node);
    std::cout << "x = " << x << std::endl;
    std::cout << "y = " << y << std::endl;
    std::cout<< "f = 2x+y^2 = " << res[0] << std::endl;
    std::cout<< "df/dx= " << gx << std::endl;
    std::cout<< "df/dy= " << gy << std::endl;
}