#include "catch2/catch.hpp"
#include "data_node.hpp"
#include "i_node.hpp"
#include "tensor.hpp"
#include <iostream>
#include <nlohmann/json.hpp>

class A
{
public:
    int a;
    double b;
};

namespace nlohmann
{
template <>
struct adl_serializer<A>
{
    static void to_json(json& j, const A& a)
    {
        j = json{{"a", a.a}, {"b", a.b}};
    }

    static void from_json(const json& j, A& a)
    {
        j.at("a").get_to(a.a);
        j.at("b").get_to(a.b);
    }
};
} // namespace nlohmann

TEST_CASE("a test", "[a_test]")
{
    nlohmann::json js;
    A a{5, 1.12};
    js["A"] = a;
    CHECK(js["A"]["a"] == 5);
    CHECK(js["A"]["b"] == a.b);
};

TEST_CASE("tensor serialize", "[tensor_serialize]")
{
    g::Tensor t({2, 3}, {1, 2, 3, 4, 5, 6});

    nlohmann::json js = t;
    g::Tensor b(js.get<g::Tensor>());
    CHECK(js["shape"][0] == 1);
    CHECK(js["shape"][1] == 1);
    CHECK(js["shape"][2] == 2);
    CHECK(js["shape"][3] == 3);
    CHECK(b == t);
}

TEST_CASE("node serialize", "[node_serialize]")
{
    std::shared_ptr<g::DataNode> x = std::make_shared<g::DataNode>("xx");
    std::shared_ptr<g::DataNode> A = std::make_shared<g::DataNode>("AA");
    x->set_value(g::Tensor({1}));
    A->set_value(g::Tensor({1.2}));
    std::shared_ptr<g::INode> a1 = g::INode::factory("MultNode", "1");
    g::set_dep(a1, {A, x});

    nlohmann::json js = a1;
    nlohmann::json js1 = static_cast<g::INode::PNode>(x);
    CHECK(js["classname"] == "MultNode");
    CHECK(js1["classname"] == "DataNode");
}
