#include "arithmetic_nodes.hpp"
#include "i_functional_node.hpp"
#include "input_node.hpp"
#include "model.hpp"
#include "power_nodes.hpp"
#include <catch2/catch.hpp>
#include <iostream>
#include <fstream>

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
}
TEST_CASE("serialization", "[3]")
{
    g::set_log_debug();
    std::shared_ptr<INode> x = INode::factory("InputNode", "x");
    std::shared_ptr<INode> y = INode::factory("InputNode", "y");
    std::shared_ptr<INode> A = INode::factory("InputNode", "A");

    std::shared_ptr<INode> a1 = INode::factory("MultNode", "a1");
    a1->set_prev_nodes({A, x});
    std::shared_ptr<INode> a2 = INode::factory("PlusNode", "a2");
    a2->set_prev_nodes({a1, x});
    std::shared_ptr<INode> f = INode::factory("SqrNode", "f");
    f->set_prev_nodes({a2});
    std::shared_ptr<INode> g = INode::factory("MultNode", "g");
    g->set_prev_nodes({a2, a2});

    Model model({x, y, A}, {f,g});
    std::vector<double> v1 = model.compute({2, 3, 1.2});
    model.save("a.json");
    Model model2 = Model::load("a.json");
    std::vector<double> v2 = model2.compute({2, 3, 1.2});
    CHECK(v1 == v2);
}

TEST_CASE("serialization2", "[4]")
{
    g::set_log_debug();
    std::shared_ptr<INode> x = INode::factory("InputNode", {});
    std::shared_ptr<INode> y = INode::factory("InputNode", {});
    std::shared_ptr<INode> A = INode::factory("InputNode", {});
    std::shared_ptr<INode> B = INode::factory("InputNode", {});

    std::shared_ptr<INode> a1 = INode::factory("MultNode", "aa1");
    a1->set_prev_nodes({A, x});
    std::shared_ptr<INode> a2 = INode::factory("PlusNode", "aa2");
    a2->set_prev_nodes({a1, B});
    std::shared_ptr<INode> a3 = INode::factory("MinusNode", "aa3");
    a3->set_prev_nodes({y, a2});
    std::shared_ptr<INode> f = INode::factory("SqrNode", "ff");
    f->set_prev_nodes({a3});
    
    Model model({x, y, A, B}, {f});
    std::vector<double> v1 = model.compute({2, 3,1,5});
    model.save("a.json");
    Model model2 = Model::load("a.json");
    std::vector<double> v2 = model2.compute({2, 3,1,5});
    model.save("b.json");
    CHECK(v1 == v2);
    CHECK(load_json("a.json") == load_json("b.json"));
}