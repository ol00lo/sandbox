#include "arithmetic_nodes.hpp"
#include "i_functional_node.hpp"
#include "data_node.hpp"
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
    g::set_log_debug();
    std::shared_ptr<INode> x = INode::factory("DataNode", "x");
    std::shared_ptr<INode> y = INode::factory("DataNode", "y");
    std::shared_ptr<INode> A = INode::factory("DataNode", "A");
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
    auto res = std::vector<double>(v0[0]);
    CHECK(res[0] == Approx(0.04));
    std::vector<Tensor> v1 = model.compute({Tensor({2}), Tensor({-3})});
    auto res1 = std::vector<double>(v1[0]);
    CHECK(res1[0] == Approx(0.36));
    model.save("a.json");
    Model model2 = Model::load("a.json");
    std::vector<Tensor> v2 = model2.compute({Tensor({2}), Tensor({-3})});
    auto res2 = std::vector<double>(v2[0]);
    CHECK(res1 == res2);
}

TEST_CASE("serialization2", "[4]")
{
    g::set_log_debug();
    std::shared_ptr<INode> x = INode::factory("DataNode", {});
    std::shared_ptr<INode> y = INode::factory("DataNode", {});
    std::shared_ptr<INode> A = INode::factory("DataNode", {});
    std::shared_ptr<INode> B = INode::factory("DataNode", {});
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
    CHECK(std::vector<double>(v1[0]) == std::vector<double>(v2[0]));
    CHECK(load_json("a.json") == load_json("b.json"));
}