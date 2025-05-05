#include "arithmetic_nodes.hpp"
#include "data_node.hpp"
#include "i_functional_node.hpp"
#include "model.hpp"
#include "power_nodes.hpp"
#include <catch2/catch.hpp>
#include <fstream>
#include <iostream>

using namespace g;
namespace
{
nlohmann::json load_json(const std::string& filename)
{
    std::ifstream file(filename);
    nlohmann::json j;
    if (file.is_open())
    {
        file >> j;
    }
    return j;
}
} // namespace

TEST_CASE("serialization", "[3]")
{
    std::shared_ptr<DataNode> x = std::make_shared<DataNode>("xxx");
    std::shared_ptr<DataNode> y = std::make_shared<DataNode>("yyy");
    std::shared_ptr<DataNode> A = std::make_shared<DataNode>("AAA");
    x->set_value(Tensor({1}));
    y->set_value(Tensor({-1}));
    A->set_value(Tensor({1.2}));
    std::shared_ptr<INode> a1 = INode::factory("MultNode", "a1");
    g::set_dep(a1, {A, x});
    std::shared_ptr<INode> a2 = INode::factory("PlusNode", "a2");
    g::set_dep(a2, {a1, y});
    std::shared_ptr<INode> f = INode::factory("SqrNode", "f");
    g::set_dep(f, {a2});
    std::shared_ptr<INode> g = INode::factory("MultNode", "g");
    g::set_dep(g, {a2, a2});

    Model model({x, y}, {f, g});
    std::vector<Tensor> v0 = model.compute({});
    CHECK(v0[0][0] == Approx(0.04));
    std::vector<Tensor> v1 = model.compute({Tensor({2}), Tensor({-3})});
    CHECK(v1[0][0] == Approx(0.36));
    model.save("a.json");

    Model model2 = Model::load("a.json");
    std::vector<Tensor> v2 = model2.compute({Tensor({2}), Tensor({-3})});
    CHECK(v1[0] == v2[0]);
    CHECK(v1[1] == v2[1]);
}

TEST_CASE("serialization2", "[4]")
{
    std::shared_ptr<DataNode> x = std::make_shared<DataNode>("");
    std::shared_ptr<DataNode> y = std::make_shared<DataNode>("");
    std::shared_ptr<DataNode> A = std::make_shared<DataNode>("");
    std::shared_ptr<DataNode> B = std::make_shared<DataNode>("");
    A->set_value(Tensor({1}));
    B->set_value(Tensor({5}));
    std::shared_ptr<INode> a1 = INode::factory("MultNode", "aa1");
    g::set_dep(a1, {A, x});
    std::shared_ptr<INode> a2 = INode::factory("PlusNode", "aa2");
    g::set_dep(a2, {a1, B});
    std::shared_ptr<INode> a3 = INode::factory("MinusNode", "aa3");
    g::set_dep(a3, {y, a2});
    std::shared_ptr<INode> f = INode::factory("SqrNode", "ff");
    g::set_dep(f, {a3});

    Model model({x, y}, {f});
    std::vector<Tensor> v1 = model.compute({Tensor({2}), Tensor({3})});
    model.save("a.json");
    Model model2 = Model::load("a.json");
    std::vector<Tensor> v2 = model2.compute({Tensor({2}), Tensor({3})});
    model.save("b.json");
    CHECK(v1[0] == v2[0]);
    CHECK(load_json("a.json") == load_json("b.json"));
}

TEST_CASE("trigonometric nodes test", "[tn_test]")
{
    std::shared_ptr<DataNode> x = std::make_shared<DataNode>("");
    std::shared_ptr<INode> a = INode::factory("SinNode", "");
    g::set_dep(a, {x});
    Model model({x}, {a});
    std::vector<Tensor> v1 = model.compute({Tensor({0})});
    model.save("a.json");
    Model model2 = Model::load("a.json");
    std::vector<Tensor> v2 = model2.compute({Tensor({0})});
    model.save("b.json");
    CHECK(v1[0] == v2[0]);
    CHECK(load_json("a.json") == load_json("b.json"));

    std::shared_ptr<DataNode> y = std::make_shared<DataNode>("");
    std::shared_ptr<INode> b = INode::factory("CosNode", "");
    g::set_dep(b, {y});
    Model model1({y}, {b});
    std::vector<Tensor> v3 = model1.compute({Tensor({3.14159 / 2})});
    CHECK(v3[0][0] == Approx(v1[0][0]).margin(1e-5));
}

TEST_CASE("same nodes name", "[n_name_test]")
{
    std::shared_ptr<DataNode> x = std::make_shared<DataNode>("x");
    std::shared_ptr<INode> y = INode::factory("SinNode", "x");
    g::set_dep(y, {x});
    CHECK_THROWS(Model({x}, {y}));
}